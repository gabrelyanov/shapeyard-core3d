#pragma once
// Permanent write-ahead data component. This numeric-only store owns no native
// document, context, geometry or executable admission. Reservation is required,
// but not sufficient, for future owner-issued typed-command admission. Disk work
// must run off main; no public mutation/query is wired by this component.
#include <CommonCrypto/CommonDigest.h>
#include <sys/file.h>
#if DEBUG
#include <dirent.h>
#endif
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
// Frozen V1 wire bound, not a V2 reservation capacity.
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

namespace detail {
// V2 is a persistent binary Patricia index. Only the 80-byte root is replaced.
// Every immutable page is addressed by SHA-256; an absent referenced page is
// corruption, never evidence of an unseen request. Legacy codec bounds above
// remain exact and are used only to read/migrate the original V1 catalog.
constexpr std::size_t IndexRootBytes=80, MaximumPageBytes=136, MaximumPathPages=129;
inline bool Zero(const Digest& d)noexcept{return std::all_of(d.begin(),d.end(),[](auto x){return x==0;});}
inline unsigned Bit(const UUID& id,unsigned bit)noexcept{return (id[bit/8]>>(7-bit%8))&1;}
inline unsigned Difference(const UUID&a,const UUID&b,unsigned limit=128)noexcept {
    unsigned bit=0;while(bit<limit&&Bit(a,bit)==Bit(b,bit))++bit;return bit;
}
inline UUID Prefix(UUID id,unsigned bit)noexcept{for(unsigned i=bit;i<128;++i)id[i/8]&=std::uint8_t(~(1u<<(7-i%8)));return id;}
inline void U64(std::vector<std::uint8_t>&b,std::uint64_t n){for(unsigned i=0;i<8;++i)b.push_back(std::uint8_t(n>>(8*i)));}
inline std::uint64_t U64(const std::uint8_t*p)noexcept{std::uint64_t n=0;for(unsigned i=0;i<8;++i)n|=std::uint64_t(p[i])<<(8*i);return n;}
inline Digest Hash(const std::vector<std::uint8_t>&b)noexcept {
    Digest d{};if(b.size()>MaximumBytes||!CC_SHA256(b.data(),CC_LONG(b.size()),d.data()))d.fill(0);return d;
}
inline std::string Hex(const Digest&d){static constexpr char h[]="0123456789abcdef";std::string s;for(auto x:d){s+=h[x>>4];s+=h[x&15];}return s;}
struct IndexRoot { Digest hash{};std::uint64_t count=0; };
inline std::vector<std::uint8_t> RootBytes(const IndexRoot&r){
    std::vector<std::uint8_t>b{'S','Y','N','T',2,0,0,0};U64(b,r.count);b.insert(b.end(),r.hash.begin(),r.hash.end());
    const auto h=Hash(b);b.insert(b.end(),h.begin(),h.end());return b;
}
inline bool ReadRoot(const std::vector<std::uint8_t>&b,IndexRoot&r)noexcept {
    r={};if(b.size()!=IndexRootBytes||std::memcmp(b.data(),"SYNT\2\0\0\0",8))return false;
    try{const std::vector<std::uint8_t>body(b.begin(),b.end()-32);const auto h=Hash(body);
        if(Zero(h)||!std::equal(h.begin(),h.end(),b.end()-32))return false;
        r.count=U64(b.data()+8);std::copy_n(b.begin()+16,32,r.hash.begin());return (r.count==0)==Zero(r.hash);
    }catch(...){r={};return false;}
}
struct Page {
    unsigned bit=128;UUID prefix{};Key key{};
    std::array<IndexRoot,2> children{};
    std::uint64_t count()const noexcept{return bit==128?1:children[0].count+children[1].count;}
};
inline std::vector<std::uint8_t> PageBytes(const Page&p){
    if(p.bit==128){std::vector<std::uint8_t>b{'S','Y','N','L',1,0,0,0};
        const auto append=[&](const auto&f){b.insert(b.end(),f.begin(),f.end());};
        append(p.key.accountScope);append(p.key.document);append(p.key.request);append(p.key.command);append(p.key.execution);return b;}
    std::vector<std::uint8_t>b{'S','Y','N','B',1,std::uint8_t(p.bit),0,0};b.insert(b.end(),p.prefix.begin(),p.prefix.end());
    for(const auto&c:p.children){U64(b,c.count);b.insert(b.end(),c.hash.begin(),c.hash.end());}return b;
}
inline bool ReadPageBytes(const std::vector<std::uint8_t>&b,Page&p)noexcept {
    p={};if(b.size()<8||b[4]!=1||b[6]||b[7])return false;
    if(b.size()==136&&!std::memcmp(b.data(),"SYNL",4)&&b[5]==0){
        std::size_t at=8;const auto take=[&](auto&f){std::copy_n(b.begin()+at,f.size(),f.begin());at+=f.size();};
        take(p.key.accountScope);take(p.key.document);take(p.key.request);take(p.key.command);take(p.key.execution);
        p.prefix=p.key.request;return Valid(p.key);
    }
    if(b.size()!=104||std::memcmp(b.data(),"SYNB",4)||b[5]>=128)return false;
    p.bit=b[5];std::copy_n(b.begin()+8,16,p.prefix.begin());if(Prefix(p.prefix,p.bit)!=p.prefix)return false;
    std::size_t at=24;for(auto&c:p.children){c.count=U64(b.data()+at);at+=8;std::copy_n(b.begin()+at,32,c.hash.begin());at+=32;
        if(!c.count||Zero(c.hash))return false;}
    return p.children[0].hash!=p.children[1].hash&&p.children[0].count<=UINT64_MAX-p.children[1].count;
}
inline FD OpenPages(int root,bool create)noexcept {
    if(create&&::mkdirat(root,"pages-v2",0700)!=0&&errno!=EEXIST)return FD();
    FD pages(::openat(root,"pages-v2",O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
    if(!Owned(pages.value,true,0700)||(create&&!DirectorySync(root)))return FD();return pages;
}
inline FD OpenShard(int pages,const std::string&hex,bool create) {
    const auto shard=hex.substr(0,2);
    if(create&&::mkdirat(pages,shard.c_str(),0700)!=0&&errno!=EEXIST)return FD();
    FD dir(::openat(pages,shard.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
    if(!Owned(dir.value,true,0700)||(create&&!DirectorySync(pages)))return FD();return dir;
}
inline bool LoadPage(int pages,const IndexRoot&ref,Page&p) {
    if(!ref.count||Zero(ref.hash))return false;
    const auto hex=Hex(ref.hash);auto shard=OpenShard(pages,hex,false);if(shard.value<0)return false;
    FD file(::openat(shard.value,hex.c_str()+2,O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));std::vector<std::uint8_t>b;
    return Owned(file.value,false,0600)&&ReadAll(file.value,MaximumPageBytes,b)&&Hash(b)==ref.hash
        &&ReadPageBytes(b,p)&&p.count()==ref.count;
}
// No mutable page replacement, truncation or page reclamation. Existing pages
// must match in full, including their hash and strict ownership/link/mode rules.
inline bool SavePage(int pages,const Page&p,IndexRoot&ref) {
    const auto b=PageBytes(p);const auto hash=Hash(b);Page check;
    if(Zero(hash)||!ReadPageBytes(b,check))return false;
    const auto hex=Hex(hash);auto shard=OpenShard(pages,hex,true);if(shard.value<0)return false;
    FD file(::openat(shard.value,hex.c_str()+2,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK,0600));
    if(file.value>=0){
        if(!Owned(file.value,false,0600)||!WriteAll(file.value,b.data(),b.size())||!FileSync(file.value)||!DirectorySync(shard.value))return false;
    }else if(errno!=EEXIST)return false;
    detail::FD durable(::openat(shard.value,hex.c_str()+2,O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
    std::vector<std::uint8_t>actual;
    if(!Owned(durable.value,false,0600)||!ReadAll(durable.value,MaximumPageBytes,actual)||actual!=b
        ||!FileSync(durable.value)||!DirectorySync(shard.value))return false;
    ref={hash,p.count()};Page reread;return LoadPage(pages,ref,reread)&&PageBytes(reread)==b;
}
// One-time conversion uses the validated, UUID-sorted V1 catalog directly.
// At most 128 leaves +127 branches; no path-copy intermediate generations.
inline bool BuildLegacy(int pages,const std::vector<Key>&keys,std::size_t begin,std::size_t end,IndexRoot&ref,unsigned depth=0) {
    if(keys.size()>MaximumRecords||begin>end||end>keys.size()||depth>128)return false;
    if(begin==end){ref={};return true;}
    if(end-begin==1){Page leaf;leaf.key=keys[begin];leaf.prefix=leaf.key.request;return SavePage(pages,leaf,ref);}
    const auto bit=Difference(keys[begin].request,keys[end-1].request);if(bit>=128)return false;
    auto split=begin;while(split<end&&!Bit(keys[split].request,bit))++split;if(split==begin||split==end)return false;
    Page branch;branch.bit=bit;branch.prefix=Prefix(keys[begin].request,bit);
    return BuildLegacy(pages,keys,begin,split,branch.children[0],depth+1)
        &&BuildLegacy(pages,keys,split,end,branch.children[1],depth+1)
        &&SavePage(pages,branch,ref)&&ref.count==end-begin;
}

struct Step{Page page;unsigned side=0;};
// At most 128 branches and one leaf. Every followed edge verifies increasing
// split position, exact prefix/direction and subtree count, in addition to hash.
inline bool Find(int pages,const IndexRoot&root,const UUID&id,std::vector<Step>&path,Page&terminal,bool&empty) {
    path.clear();empty=root.count==0;if(empty)return Zero(root.hash);
    auto ref=root;int parentBit=-1;UUID parentPrefix{};unsigned parentSide=0;
    for(std::size_t visited=0;visited<MaximumPathPages;++visited){
        Page p;if(!LoadPage(pages,ref,p)||int(p.bit)<=parentBit)return false;
        if(parentBit>=0&&(Difference(p.prefix,parentPrefix,unsigned(parentBit))!=unsigned(parentBit)
            ||Bit(p.prefix,unsigned(parentBit))!=parentSide))return false;
        if(Difference(id,p.prefix,p.bit)!=p.bit||p.bit==128){terminal=p;return true;}
        const auto side=Bit(id,p.bit);path.push_back({p,side});parentBit=int(p.bit);parentPrefix=p.prefix;parentSide=side;ref=p.children[side];
    }return false;
}
inline bool Insert(int pages,IndexRoot&root,const Key&key) {
    std::vector<Step>path;Page old;bool empty=false;if(!Find(pages,root,key.request,path,old,empty))return false;
    if(!empty&&old.bit==128&&old.key.request==key.request)return old.key==key;
    if(root.count==UINT64_MAX)return false;
    Page leaf;leaf.key=key;leaf.prefix=key.request;IndexRoot added;if(!SavePage(pages,leaf,added))return false;
    if(!empty){
        const unsigned bit=Difference(key.request,old.prefix,old.bit);if(bit>=128)return false;
        Page branch;branch.bit=bit;branch.prefix=Prefix(key.request,bit);const auto side=Bit(key.request,bit);
        branch.children[side]=added;branch.children[1-side]={Hash(PageBytes(old)),old.count()};
        if(!SavePage(pages,branch,added))return false;
    }
    for(auto i=path.rbegin();i!=path.rend();++i){i->page.children[i->side]=added;if(!SavePage(pages,i->page,added))return false;}
    if(added.count!=root.count+1)return false;root=added;return true;
}
#if DEBUG
// Only native-created disposable qualification roots use this cleanup seam.
// Production has no page deletion/eviction/reset path. Refuse unknown entries,
// symlinks, hard links, modes and more than 32768 page files rather than traverse
// arbitrary data. The per-request index itself has no such test-cleanup bound.
inline bool RemoveTestingPages(int root)noexcept {
    if(Absent(root,"pages-v2"))return true;
    try{
        auto pages=OpenPages(root,false);if(pages.value<0)return false;
        struct Listing { DIR* value=nullptr;explicit Listing(int fd){const int copy=::dup(fd);if(copy>=0){value=::fdopendir(copy);if(!value)::close(copy);}}~Listing(){if(value)::closedir(value);} };
        const auto hex=[](const std::string&s,std::size_t count){return s.size()==count&&std::all_of(s.begin(),s.end(),[](char c){return(c>='0'&&c<='9')||(c>='a'&&c<='f');});};
        Listing shards(pages.value);if(!shards.value)return false;std::size_t count=0,dirs=0;
        for(;;){errno=0;const auto entry=::readdir(shards.value);if(!entry){if(errno)return false;break;}
            const std::string name=entry->d_name;if(name=="."||name=="..")continue;if(!hex(name,2)||++dirs>256)return false;
            FD shard(::openat(pages.value,name.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));if(!Owned(shard.value,true,0700))return false;
            Listing files(shard.value);if(!files.value)return false;
            for(;;){errno=0;const auto item=::readdir(files.value);if(!item){if(errno)return false;break;}
                const std::string leaf=item->d_name;if(leaf=="."||leaf=="..")continue;if(!hex(leaf,62)||++count>32768)return false;
                FD file(::openat(shard.value,leaf.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
                if(!Owned(file.value,false,0600)||::unlinkat(shard.value,leaf.c_str(),0)!=0)return false;
            }
            if(::unlinkat(pages.value,name.c_str(),AT_REMOVEDIR)!=0)return false;
        }
        return ::unlinkat(root,"pages-v2",AT_REMOVEDIR)==0;
    }catch(...){return false;}
}
#endif

}

class Store final {
public:
    Store()=default;
#if DEBUG
    enum class Fault { None, AfterPartialPendingWrite, AfterPendingSync, AfterRename, AfterDirectorySync,
        AfterIndexPages, AfterLegacyArchive };
    static Store ForTesting(std::string privateParent){Store s;s.testParent=std::move(privateParent);return s;}
    void setNextFault(Fault f)noexcept{nextFault=f;}
#endif
    Reservation reserve(const Key& key)noexcept {return perform(key,true).reservation;}
    Presence lookup(const Key& key)noexcept {return perform(key,false).presence;}
private:
    struct Result {Reservation reservation=Reservation::Unavailable;Presence presence=Presence::Unavailable;};
#if DEBUG
    std::string testParent;Fault nextFault=Fault::None;
    bool fail(Fault f)noexcept{if(nextFault!=f)return false;nextFault=Fault::None;return true;}
#endif
    // Persist the global uncertainty marker BEFORE any new immutable pages or
    // migration archive. Existing V1 binaries already refuse this marker.
    detail::FD begin(int root)noexcept {
        detail::FD pending(::openat(root,"pending",O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK,0600));
        if(!detail::Owned(pending.value,false,0600)||!detail::DirectorySync(root))return detail::FD();return pending;
    }
    bool publish(int root,int pending,const detail::IndexRoot&index) {
        const auto bytes=detail::RootBytes(index);
#if DEBUG
        if(fail(Fault::AfterPartialPendingWrite)){
            (void)detail::WriteAll(pending,bytes.data(),3);(void)detail::FileSync(pending);return false;}
#endif
        if(!detail::WriteAll(pending,bytes.data(),bytes.size())||!detail::FileSync(pending))return false;
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
        detail::FD committed(::openat(root,"records",O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
        std::vector<std::uint8_t>actual;detail::IndexRoot checked;
        return detail::Owned(committed.value,false,0600)&&detail::ReadAll(committed.value,detail::IndexRootBytes,actual)
            &&actual==bytes&&detail::ReadRoot(actual,checked);
    }
    bool archive(int root,const std::vector<std::uint8_t>&bytes) {
        detail::FD file(::openat(root,"legacy-v1",O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK,0600));
        if(file.value>=0){if(!detail::Owned(file.value,false,0600)||!detail::WriteAll(file.value,bytes.data(),bytes.size())||!detail::FileSync(file.value))return false;}
        else if(errno!=EEXIST)return false;
        detail::FD read(::openat(root,"legacy-v1",O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));std::vector<std::uint8_t>actual;
        return detail::Owned(read.value,false,0600)&&detail::ReadAll(read.value,MaximumBytes,actual)&&actual==bytes
            &&detail::FileSync(read.value)&&detail::DirectorySync(root);
    }
    Result perform(const Key& key,bool write)noexcept {
        try {
            if(::pthread_main_np()||!Valid(key))return {};
            std::unique_lock<std::mutex> processLock(detail::ProcessMutex(),std::try_to_lock);
            if(!processLock.owns_lock())return {Reservation::Busy,Presence::Busy};
            detail::FD root(
#if DEBUG
                !testParent.empty()?detail::OpenRootInParent(testParent):
#endif
                detail::OpenNativeRoot());
            if(root.value<0)return {};
            detail::FD lock(::openat(root.value,"lock",O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK,0600));
            if(!detail::Owned(lock.value,false,0600))return {};
            if(::flock(lock.value,LOCK_EX|LOCK_NB)!=0){
                if(errno==EWOULDBLOCK||errno==EAGAIN)return {Reservation::Busy,Presence::Busy};return {};}
            // Closing the descriptor releases the lock on every return path.
            if(!detail::Absent(root.value,"pending"))return {};
            std::vector<std::uint8_t>marker;if(!detail::ReadAll(lock.value,8,marker))return {};
            const std::vector<std::uint8_t>initialized{'S','Y','N','I',1,0,0,0};
            bool newlyReserved=false;
            if(marker.empty()){
                // Never infer a new installation from a missing marker if any
                // known root generation, migration archive or index survives.
                if(!detail::Absent(root.value,"records")||!detail::Absent(root.value,"pages-v2")||!detail::Absent(root.value,"legacy-v1")
                    ||!detail::DirectorySync(root.value)||!detail::WriteAll(lock.value,initialized.data(),initialized.size())
                    ||!detail::FileSync(lock.value)||!detail::DirectorySync(root.value))return {};
                auto pending=begin(root.value);if(pending.value<0)return {};
                auto pages=detail::OpenPages(root.value,true);if(pages.value<0)return {};
                detail::IndexRoot initial;
                if(write){if(!detail::Insert(pages.value,initial,key))return {};newlyReserved=true;}
#if DEBUG
                if(fail(Fault::AfterIndexPages))return {};
#endif
                if(!publish(root.value,pending.value,initial))return {};
            }else if(marker!=initialized)return {};
            detail::FD file(::openat(root.value,"records",O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK));
            std::vector<std::uint8_t>bytes;detail::IndexRoot index;
            if(!detail::Owned(file.value,false,0600)||!detail::ReadAll(file.value,MaximumBytes,bytes))return {};
            if(!detail::ReadRoot(bytes,index)){
                // Only a fully validated V1 generation is eligible. Unknown,
                // malformed or partial V2 data never falls back to an archive.
                std::vector<Key>legacy;if(!Decode(bytes,legacy))return {};
                auto pending=begin(root.value);if(pending.value<0||!archive(root.value,bytes))return {};
#if DEBUG
                if(fail(Fault::AfterLegacyArchive))return {};
#endif
                auto pages=detail::OpenPages(root.value,true);if(pages.value<0)return {};
                if(!detail::BuildLegacy(pages.value,legacy,0,legacy.size(),index))return {};
                // A new reservation and migration share ONE publication. Even
                // an uncertain reply after rename must retain this new UUID.
                if(write&&std::none_of(legacy.begin(),legacy.end(),[&](const Key&previous){return previous.request==key.request;})){
                    if(!detail::Insert(pages.value,index,key))return {};newlyReserved=true;
                }
                // Independent membership/binding readback for EVERY migrated
                // key, at most the frozen V1 limit, before publishing V2.
                for(const auto&previous:legacy){std::vector<detail::Step>path;detail::Page leaf;bool empty;
                    if(!detail::Find(pages.value,index,previous.request,path,leaf,empty)||empty||leaf.bit!=128||!(leaf.key==previous))return {};}
#if DEBUG
                if(fail(Fault::AfterIndexPages))return {};
#endif
                if(index.count!=legacy.size()+(newlyReserved?1:0)||!publish(root.value,pending.value,index))return {};
            }
            auto pages=detail::OpenPages(root.value,false);if(pages.value<0)return {};
            std::vector<detail::Step>path;detail::Page terminal;bool empty;
            if(!detail::Find(pages.value,index,key.request,path,terminal,empty))return {};
            if(!empty&&terminal.bit==128&&terminal.key.request==key.request){
                const bool same=terminal.key==key;return {same?(newlyReserved?Reservation::Reserved:Reservation::AlreadyReserved):Reservation::Conflict,same?Presence::Match:Presence::Conflict};}
            if(!write)return {Reservation::Unavailable,Presence::Absent};
            // Fixed-width arithmetic exhaustion is refused; this is not the old
            //128-record product cap and does not evict any permanent identity.
            if(index.count==UINT64_MAX)return {Reservation::Capacity,Presence::Absent};
            auto pending=begin(root.value);if(pending.value<0||!detail::Insert(pages.value,index,key))return {};
#if DEBUG
            if(fail(Fault::AfterIndexPages))return {};
#endif
            if(!publish(root.value,pending.value,index))return {};
            if(!detail::Find(pages.value,index,key.request,path,terminal,empty)||empty||terminal.bit!=128||!(terminal.key==key))return {};
            return {Reservation::Reserved,Presence::Match};
        }catch(...){return {};}
    }
};
} // namespace core3d::tombstone
