#pragma once
// Permanent write-ahead data component. This numeric-only store owns no native
// document, context, geometry or executable admission. Reservation is required,
// but not sufficient, for future owner-issued typed-command admission. Disk work
// must run off main; no public mutation/query is wired by this component.
#include <CommonCrypto/CommonDigest.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace core3d::tombstone {
using Digest=std::array<std::uint8_t,32>;
using UUID=std::array<std::uint8_t,16>;
constexpr std::size_t MaximumRecords=128, RecordBytes=128, MaximumBytes=40+MaximumRecords*RecordBytes;
struct Key {
    Digest accountScope{},command{},execution{};
    UUID document{},request{};
    bool operator==(const Key& b)const noexcept{return accountScope==b.accountScope&&command==b.command&&execution==b.execution&&document==b.document&&request==b.request;}
};
enum class Reservation { Reserved, AlreadyReserved, Conflict, Capacity, Busy, Unavailable };
enum class Presence { Absent, Match, Conflict, Busy, Unavailable };
// There is deliberately no 'delete', 'reset', 'retry', 'cancel reservation',
// TTL/eviction, or phase that makes a previously seen UUID executable again.
inline bool Valid(const Key& key)noexcept {
    auto nonzero=[](const auto& x){return std::any_of(x.begin(),x.end(),[](auto v){return v!=0;});};
    return nonzero(key.accountScope)&&nonzero(key.command)&&nonzero(key.execution)&&nonzero(key.document)&&nonzero(key.request);
}
inline bool Encode(std::vector<Key> records,std::vector<std::uint8_t>& output)noexcept {
    output.clear();try {
        if(records.size()>MaximumRecords)return false;
        std::sort(records.begin(),records.end(),[](const Key&a,const Key&b){return a.request<b.request;});
        std::vector<std::uint8_t> bytes{'S','Y','N','T',1,0,std::uint8_t(records.size()),std::uint8_t(records.size()>>8)};
        UUID previous{};
        const auto append=[&](const auto& field){bytes.insert(bytes.end(),field.begin(),field.end());};
        for(const auto& key:records){if(!Valid(key)||!(previous<key.request))return false;previous=key.request;
            append(key.accountScope);append(key.document);append(key.request);append(key.command);append(key.execution);}
        Digest digest{};if(!CC_SHA256(bytes.data(),CC_LONG(bytes.size()),digest.data()))return false;
        append(digest);output=std::move(bytes);return true;
    }catch(...){return false;}
}
inline bool Decode(const std::vector<std::uint8_t>& bytes,std::vector<Key>& output)noexcept {
    output.clear();try {
        if(bytes.size()<40||bytes.size()>MaximumBytes||std::memcmp(bytes.data(),"SYNT",4)||bytes[4]!=1||bytes[5]!=0)return false;
        const auto count=std::size_t(bytes[6])+(std::size_t(bytes[7])<<8);
        if(count>MaximumRecords||bytes.size()!=40+count*RecordBytes)return false;
        Digest digest{};if(!CC_SHA256(bytes.data(),CC_LONG(bytes.size()-32),digest.data())
            ||!std::equal(digest.begin(),digest.end(),bytes.end()-32))return false;
        std::size_t cursor=8;std::vector<Key> records;UUID previous{};
        const auto take=[&](auto& field){std::copy_n(bytes.begin()+cursor,field.size(),field.begin());cursor+=field.size();};
        for(std::size_t i=0;i<count;++i){Key key;take(key.accountScope);take(key.document);take(key.request);take(key.command);take(key.execution);
            if(!Valid(key)||!(previous<key.request))return false;previous=key.request;records.push_back(key);}
        output=std::move(records);return true;
    }catch(...){return false;}
}

namespace detail {
struct FD {
    int value=-1;
    explicit FD(int v=-1):value(v){}
    ~FD(){if(value>=0)::close(value);}
    FD(const FD&)=delete;FD&operator=(const FD&)=delete;
    FD(FD&&b)noexcept:value(std::exchange(b.value,-1)){}
};
inline std::mutex& ProcessMutex(){static std::mutex mutex;return mutex;}
inline bool Owned(int fd,bool directory,mode_t permissions)noexcept {
    struct stat s{};return fd>=0&&::fstat(fd,&s)==0&&s.st_uid==::geteuid()
        &&(directory?S_ISDIR(s.st_mode):S_ISREG(s.st_mode))&&(s.st_mode&07777)==permissions
        &&(directory||s.st_nlink==1);
}
inline bool FileSync(int fd)noexcept {
    int rc;do{rc=::fsync(fd);}while(rc!=0&&errno==EINTR);if(rc!=0)return false;
#if defined(__APPLE__)
    do{rc=::fcntl(fd,F_FULLFSYNC);}while(rc!=0&&errno==EINTR);
    if(rc!=0)return false; // no silent weakening of the durability boundary
#endif
    return true;
}
inline bool DirectorySync(int fd)noexcept{int rc;do{rc=::fsync(fd);}while(rc!=0&&errno==EINTR);return rc==0;}
inline bool WriteAll(int fd,const std::uint8_t* bytes,std::size_t count)noexcept {
    std::size_t offset=0;while(offset<count){const auto n=::write(fd,bytes+offset,count-offset);
        if(n<0&&errno==EINTR)continue;if(n<=0)return false;offset+=std::size_t(n);}return true;
}
inline bool ReadAll(int fd,std::size_t maximum,std::vector<std::uint8_t>& output)noexcept {
    output.clear();try {
        struct stat before{};if(::fstat(fd,&before)!=0||before.st_size<0||std::uint64_t(before.st_size)>maximum)return false;
        std::vector<std::uint8_t> bytes(std::size_t(before.st_size));std::size_t offset=0;
        while(offset<bytes.size()){const auto n=::pread(fd,bytes.data()+offset,bytes.size()-offset,off_t(offset));
            if(n<0&&errno==EINTR)continue;if(n<=0)return false;offset+=std::size_t(n);}
        struct stat after{};if(::fstat(fd,&after)!=0||after.st_size!=before.st_size||after.st_ino!=before.st_ino||after.st_dev!=before.st_dev)return false;
        output=std::move(bytes);return true;
    }catch(...){return false;}
}
inline bool Absent(int directory,const char* name)noexcept {
    struct stat s{};return ::fstatat(directory,name,&s,AT_SYMLINK_NOFOLLOW)!=0&&errno==ENOENT;
}
// Opens only the fixed native root beneath Application Support. Its parent is
// provided internally by the Foundation implementation, never by app callers.
// DEBUG tests alone can supply a private temporary parent directory.
int OpenNativeRoot()noexcept;
inline int OpenRootInParent(const std::string& parentPath)noexcept {
    FD parent(::open(parentPath.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW));
    struct stat p{};if(parent.value<0||::fstat(parent.value,&p)!=0||!S_ISDIR(p.st_mode)
        ||p.st_uid!=::geteuid()||(p.st_mode&0022)!=0)return -1;
    constexpr const char* name="NativeModelingRequests";
    bool created=::mkdirat(parent.value,name,0700)==0;
    if(!created&&errno!=EEXIST)return -1;
    FD root(::openat(parent.value,name,O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW));
    if(!Owned(root.value,true,0700)||!DirectorySync(parent.value))return -1;
    return std::exchange(root.value,-1);
}
}

class Store final {
public:
    Store()=default;
#if DEBUG
    enum class Fault { None, AfterPartialPendingWrite, AfterPendingSync, AfterRename, AfterDirectorySync };
    static Store ForTesting(std::string privateParent){Store s;s.testParent=std::move(privateParent);return s;}
    void setNextFault(Fault f)noexcept{nextFault=f;}
#endif
    Reservation reserve(const Key& key)noexcept {return perform(key,true).reservation;}
    Presence lookup(const Key& key)noexcept {return perform(key,false).presence;}
private:
    struct Result {Reservation reservation=Reservation::Unavailable;Presence presence=Presence::Unavailable;};
#if DEBUG
    std::string testParent;
    Fault nextFault=Fault::None;
    bool fail(Fault f)noexcept{if(nextFault!=f)return false;nextFault=Fault::None;return true;}
#endif
    // Called while both process-wide mutex and native file lock are owned.
    bool replace(int root,const std::vector<Key>& records)noexcept {
        std::vector<std::uint8_t> bytes;if(!Encode(records,bytes))return false;
        detail::FD pending(::openat(root,"pending",O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600));
        if(!detail::Owned(pending.value,false,0600))return false;
        // Persist the pending name before writing it. Any uncertain/partial
        // write survives as a fail-closed marker, never an auto-recovered retry.
        if(!detail::DirectorySync(root))return false;
#if DEBUG
        if(fail(Fault::AfterPartialPendingWrite)){
            (void)detail::WriteAll(pending.value,bytes.data(),3);
            (void)detail::FileSync(pending.value);return false;
        }
#endif
        if(!detail::WriteAll(pending.value,bytes.data(),bytes.size())||!detail::FileSync(pending.value))return false;
#if DEBUG
        if(fail(Fault::AfterPendingSync))return false;
#endif
        if(::renameat(root,"pending",root,"records")!=0)return false;
#if DEBUG
        if(fail(Fault::AfterRename))return false;
#endif
        if(!detail::DirectorySync(root))return false;
#if DEBUG
        if(fail(Fault::AfterDirectorySync))return false;
#endif
        return true;
    }
    Result perform(const Key& key,bool write)noexcept {
        Result result;
        try {
            if(::pthread_main_np()||!Valid(key))return result;
            std::unique_lock<std::mutex> processLock(detail::ProcessMutex(),std::try_to_lock);
            if(!processLock.owns_lock()){result.reservation=Reservation::Busy;result.presence=Presence::Busy;return result;}
            detail::FD root(
#if DEBUG
                !testParent.empty()?detail::OpenRootInParent(testParent):
#endif
                detail::OpenNativeRoot());
            if(root.value<0)return result;
            detail::FD lock(::openat(root.value,"lock",O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK,0600));
            if(!detail::Owned(lock.value,false,0600))return result;
            if(::flock(lock.value,LOCK_EX|LOCK_NB)!=0){
                if(errno==EWOULDBLOCK||errno==EAGAIN){result.reservation=Reservation::Busy;result.presence=Presence::Busy;}return result;}
            // Closing the descriptor releases the lock on every return path.
            if(!detail::Absent(root.value,"pending"))return result;
            std::vector<std::uint8_t> marker;
            if(!detail::ReadAll(lock.value,8,marker))return result;
            const std::vector<std::uint8_t> initialized{'S','Y','N','I',1,0,0,0};
            if(marker.empty()){
                // A surviving records file with a missing/empty marker is not
                // treated as a new installation. Loss of the whole root is an
                // explicit installation-local provenance limit.
                if(!detail::Absent(root.value,"records")||!detail::DirectorySync(root.value)
                    ||!detail::WriteAll(lock.value,initialized.data(),initialized.size())
                    ||!detail::FileSync(lock.value)||!detail::DirectorySync(root.value))return result;
                if(!replace(root.value,{}))return result;
            }else if(marker!=initialized)return result;
            detail::FD file(::openat(root.value,"records",O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
            std::vector<std::uint8_t> bytes;std::vector<Key> records;
            if(!detail::Owned(file.value,false,0600)||!detail::ReadAll(file.value,MaximumBytes,bytes)||!Decode(bytes,records))return result;
            for(const auto& previous:records)if(previous.request==key.request){
                const bool same=previous==key;result.reservation=same?Reservation::AlreadyReserved:Reservation::Conflict;
                result.presence=same?Presence::Match:Presence::Conflict;return result;}
            result.presence=Presence::Absent;
            if(!write)return result;
            if(records.size()>=MaximumRecords){result.reservation=Reservation::Capacity;return result;}
            records.push_back(key);
            if(!replace(root.value,records)){result.presence=Presence::Unavailable;return result;}
            // Reopen and validate the durable generation before acknowledging.
            detail::FD committed(::openat(root.value,"records",O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
            std::vector<std::uint8_t> expected,actual;std::vector<Key> reread;
            if(!Encode(records,expected)||!detail::Owned(committed.value,false,0600)
                ||!detail::ReadAll(committed.value,MaximumBytes,actual)||actual!=expected||!Decode(actual,reread))return {};
            result.reservation=Reservation::Reserved;result.presence=Presence::Match;return result;
        }catch(...){return {};}
    }
};
} // namespace core3d::tombstone
