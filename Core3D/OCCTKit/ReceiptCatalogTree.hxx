#pragma once
#include "ReceiptRecord.hxx"
#include <atomic>
#include <memory>
#include <new>
#include <limits>
#include <span>
#include <stdexcept>

namespace core3d::receipt::v3 {
// Proposed bounded resource policy. Old codec/effect limits are unchanged.
constexpr std::size_t WireLimit=8*1024*1024;
constexpr std::size_t DocumentMemoryLimit=64*1024*1024;
constexpr std::size_t ProcessMemoryLimit=128*1024*1024;
constexpr std::size_t MaximumRawRecord=1940;
constexpr std::size_t FixedWireBytes=136; // header/owner/archives/count/terminal SHA
template<class T> class Allocator;
template<class T> class ProcessAllocator;
class AllocationBudget final {
public:
    AllocationBudget()=default;
    AllocationBudget(const AllocationBudget&)=delete;
    AllocationBudget& operator=(const AllocationBudget&)=delete;
    std::size_t bytes()const noexcept{return live_.load();}
    static std::size_t processBytes()noexcept{return process_.load();}
private:
    template<class> friend class Allocator;
    template<class> friend class ProcessAllocator;
    std::atomic<std::size_t> live_{0};
    inline static std::atomic<std::size_t> process_{0};
    static bool Take(std::atomic<std::size_t>& counter,std::size_t n,std::size_t maximum) noexcept {
        auto prior=counter.load(std::memory_order_relaxed);
        do {if(n>maximum || prior>maximum-n)return false;}
        while(!counter.compare_exchange_weak(prior,prior+n,std::memory_order_acq_rel));return true;
    }
    static void ProcessReserve(std::size_t n){if(!Take(process_,n,ProcessMemoryLimit))throw std::bad_alloc();}
    static void ProcessRelease(std::size_t n)noexcept{process_.fetch_sub(n);}
    void reserve(std::size_t n){
        if(!Take(live_,n,DocumentMemoryLimit))throw std::bad_alloc();
        try{ProcessReserve(n);}catch(...){live_.fetch_sub(n);throw;}
    }
    void release(std::size_t n) noexcept {ProcessRelease(n);live_.fetch_sub(n);}
};
// Budget objects and their actual shared control blocks also pay the process
// pool. The per-document graph quota excludes this one small budget descriptor.
template<class T> class ProcessAllocator {
public:
    using value_type=T;
    ProcessAllocator()=default;
    template<class U> ProcessAllocator(const ProcessAllocator<U>&){}
    T* allocate(std::size_t n){
        if(n>std::numeric_limits<std::size_t>::max()/sizeof(T))throw std::bad_alloc();
        const auto bytes=n*sizeof(T);AllocationBudget::ProcessReserve(bytes);
        try{return static_cast<T*>(::operator new(bytes));}catch(...){AllocationBudget::ProcessRelease(bytes);throw;}
    }
    void deallocate(T* p,std::size_t n)noexcept{::operator delete(p);AllocationBudget::ProcessRelease(n*sizeof(T));}
    template<class U> bool operator==(const ProcessAllocator<U>&)const noexcept{return true;}
};
inline std::shared_ptr<AllocationBudget> MakeBudget(){return std::allocate_shared<AllocationBudget>(ProcessAllocator<AllocationBudget>());}
// allocate_shared rebinds this allocator to its actual control-block type. Both
// node payload AND implementation control-block bytes are charged before new.
template<class T> class Allocator {
public:
    using value_type=T;
    std::shared_ptr<AllocationBudget> budget;
    explicit Allocator(std::shared_ptr<AllocationBudget> b):budget(std::move(b)){}
    template<class U> Allocator(const Allocator<U>& b):budget(b.budget){}
    T* allocate(std::size_t n){
        if(!budget || n>std::numeric_limits<std::size_t>::max()/sizeof(T))throw std::bad_alloc();
        const auto bytes=n*sizeof(T);budget->reserve(bytes);
        try{return static_cast<T*>(::operator new(bytes));}catch(...){budget->release(bytes);throw;}
    }
    void deallocate(T* p,std::size_t n) noexcept {::operator delete(p);budget->release(n*sizeof(T));}
    template<class U> bool operator==(const Allocator<U>& b)const noexcept{return budget==b.budget;}
    template<class U> bool operator!=(const Allocator<U>& b)const noexcept{return !(*this==b);}
};
inline bool DecodeRaw(std::uint8_t codec,std::span<const std::uint8_t> raw,Record& out) {
    if((codec!=1&&codec!=2)||raw.size()<243||raw.size()>MaximumRawRecord)return false;
    std::vector<std::uint8_t> one={'S','Y','R','C',codec,0,1,0};
    one.insert(one.end(),raw.begin(),raw.end());Digest digest;
    if(!Hash(one.data(),one.size(),digest))return false;one.insert(one.end(),digest.begin(),digest.end());
    std::vector<Record> decoded;if(Decode(one,decoded)!=ReadStatus::Valid||decoded.size()!=1)return false;
    out=std::move(decoded.front());return true;
}
inline bool RawRecord(const Record& record,std::array<std::uint8_t,MaximumRawRecord>& raw,std::size_t& length){
    std::vector<std::uint8_t> encoded;if(!Encode({record},encoded)||encoded.size()<40)return false;
    length=encoded.size()-40;if(length>raw.size())return false;
    std::copy_n(encoded.begin()+8,length,raw.begin());return true;
}
class Tree final {
    struct Leaf final {
        const std::uint8_t codec;
        const std::uint16_t length;
        const std::array<std::uint8_t,MaximumRawRecord> raw;
        Leaf(std::uint8_t c,std::span<const std::uint8_t> bytes):codec(c),length(std::uint16_t(bytes.size())),raw(Copy(bytes)){}
        static std::array<std::uint8_t,MaximumRawRecord> Copy(std::span<const std::uint8_t> bytes){
            if(bytes.size()>MaximumRawRecord)throw std::invalid_argument("Receipt span");
            std::array<std::uint8_t,MaximumRawRecord> result{};std::copy(bytes.begin(),bytes.end(),result.begin());return result;
        }
    };
    struct Node;
    using NodePtr=std::shared_ptr<const Node>;
    struct Node final {
        const UUID key; // exact leaf key, or minimum descendant key
        const std::uint16_t split; //128 identifies a leaf
        const std::uint64_t count,wire,unsupported,legacy;
        const NodePtr zero,one;
        const std::shared_ptr<const Leaf> leaf;
        Node(const UUID& k,std::shared_ptr<const Leaf> l,bool supported)
            :key(k),split(128),count(1),wire(3+l->length),unsupported(supported?0:1),legacy(l->codec==1?1:0),leaf(std::move(l)){}
        Node(unsigned bit,NodePtr a,NodePtr b)
            :key(a->key),split(std::uint16_t(bit)),count(a->count+b->count),wire(a->wire+b->wire),
             unsupported(a->unsupported+b->unsupported),legacy(a->legacy+b->legacy),zero(std::move(a)),one(std::move(b)){}
    };
    const NodePtr root_;
    const std::shared_ptr<AllocationBudget> budget_;
    const UUID document_;
    const Digest legacy_,versioned_;
    static bool Bit(const UUID& k,unsigned bit){return (k[bit/8]>>(7-bit%8))&1;}
    static unsigned Difference(const UUID& a,const UUID& b){unsigned i=0;for(;i<128&&Bit(a,i)==Bit(b,i);++i){}return i;}
    static const Node* Find(const NodePtr& node,const UUID& key){
        auto n=node.get();unsigned depth=0;
        while(n&&n->split<128){if(++depth>128)return nullptr;n=(Bit(key,n->split)?n->one:n->zero).get();}
        return n&&n->key==key?n:nullptr;
    }
    NodePtr Branch(unsigned split,NodePtr a,NodePtr b)const{
        if(!a||!b||split>=128||a->split<=split||b->split<=split||Bit(a->key,split)||!Bit(b->key,split)
           ||Difference(a->key,b->key)!=split)throw std::invalid_argument("Receipt branch");
        return std::allocate_shared<Node>(Allocator<Node>(budget_),split,std::move(a),std::move(b));
    }
    NodePtr Insert(const NodePtr& n,const NodePtr& leaf,unsigned different,unsigned depth=0)const{
        if(depth>128)throw std::invalid_argument("Receipt depth");
        if(!n)return leaf;
        if(n->split>=different)return Bit(leaf->key,different)?Branch(different,n,leaf):Branch(different,leaf,n);
        if(Bit(leaf->key,n->split))return Branch(n->split,n->zero,Insert(n->one,leaf,different,depth+1));
        return Branch(n->split,Insert(n->zero,leaf,different,depth+1),n->one);
    }
    template<class F> static bool Walk(const NodePtr& n,F& f,unsigned depth=0){
        if(!n||depth>128)return false;
        if(n->split==128)return f(n->key,n->leaf->codec,std::span<const std::uint8_t>(n->leaf->raw.data(),n->leaf->length));
        return Walk(n->zero,f,depth+1)&&Walk(n->one,f,depth+1);
    }
public:
    // Private type with public constructor permits allocator construction; callers
    // cannot obtain or construct a Node or mutable path. Tree itself is immutable.
    Tree(NodePtr n,std::shared_ptr<AllocationBudget> b,UUID d,Digest l,Digest v)
        :root_(std::move(n)),budget_(std::move(b)),document_(d),legacy_(l),versioned_(v){}
    static std::shared_ptr<const Tree> Empty(const UUID& document,const Digest& legacy,const Digest& versioned,
                                            std::shared_ptr<AllocationBudget> budget={}){
        if(!Nonzero(document))throw std::invalid_argument("Receipt document");
        if(!budget)budget=MakeBudget();
        return std::allocate_shared<Tree>(Allocator<Tree>(budget),NodePtr{},budget,document,legacy,versioned);
    }
    std::uint64_t count()const noexcept{return root_?root_->count:0;}
    std::uint64_t wireBytes()const noexcept{return FixedWireBytes+(root_?root_->wire:0);}
    bool supportsAppend()const noexcept{return !root_||root_->unsupported==0;}
    std::uint64_t legacyCount()const noexcept{return root_?root_->legacy:0;}
    const UUID& document()const noexcept{return document_;}
    const Digest& legacyHash()const noexcept{return legacy_;}
    const Digest& versionedHash()const noexcept{return versioned_;}
    const std::shared_ptr<AllocationBudget>& budget()const noexcept{return budget_;}
    bool contains(const UUID& request)const noexcept{return Find(root_,request)!=nullptr;}
    bool lookup(const UUID& request,Record& result)const{
        const auto n=Find(root_,request);return n&&DecodeRaw(n->leaf->codec,{n->leaf->raw.data(),n->leaf->length},result);
    }
    template<class F> bool visit(F f)const{return !root_||Walk(root_,f);}
    bool canAppendMaximum()const noexcept{
        // Conservative pre-admission headroom policy; not an allocator ABI claim.
        // Actual rebound allocation sizes remain the final authority at Stage.
        constexpr std::size_t worstNodes=512*1024;
        return supportsAppend()&&wireBytes()<=WireLimit-(3+MaximumRawRecord)
            &&budget_->bytes()<=DocumentMemoryLimit-worstNodes
            &&AllocationBudget::processBytes()<=ProcessMemoryLimit-worstNodes;
    }
    std::shared_ptr<const Tree> appendRaw(std::uint8_t codec,std::span<const std::uint8_t> raw)const{
        Record decoded;if(!DecodeRaw(codec,raw,decoded)||decoded.key.document!=document_||contains(decoded.key.request)
            ||raw.size()+3>WireLimit-wireBytes())throw std::invalid_argument("Receipt record");
        auto payload=std::allocate_shared<Leaf>(Allocator<Leaf>(budget_),codec,raw);
        NodePtr leaf=std::allocate_shared<Node>(Allocator<Node>(budget_),decoded.key.request,payload,
            decoded.policy==0||SupportedPolicy(decoded.operation,decoded.policy));
        NodePtr next;
        if(!root_)next=leaf;
        else {auto n=root_.get();while(n->split<128)n=(Bit(decoded.key.request,n->split)?n->one:n->zero).get();
            const auto different=Difference(decoded.key.request,n->key);if(different==128)throw std::invalid_argument("Duplicate receipt");
            next=Insert(root_,leaf,different);}
        return std::allocate_shared<Tree>(Allocator<Tree>(budget_),std::move(next),budget_,document_,legacy_,versioned_);
    }
    std::shared_ptr<const Tree> append(const Record& record)const{
        if(!supportsAppend()||!SupportedPolicy(record.operation,record.policy))throw std::invalid_argument("Receipt policy");
        std::array<std::uint8_t,MaximumRawRecord> raw{};std::size_t length=0;
        if(!RawRecord(record,raw,length))throw std::invalid_argument("Receipt encode");return appendRaw(2,{raw.data(),length});
    }
    std::shared_ptr<const Tree> copyTo(std::shared_ptr<AllocationBudget> destination)const{
        auto copy=Empty(document_,legacy_,versioned_,std::move(destination));
        if(!visit([&](const UUID&,std::uint8_t codec,std::span<const std::uint8_t> raw){copy=copy->appendRaw(codec,raw);return true;}))
            throw std::invalid_argument("Receipt copy");return copy;
    }
};
inline bool ArchiveHash(const std::vector<std::uint8_t>& bytes,Digest& result){return Hash(bytes.data(),bytes.size(),result);}
inline std::shared_ptr<const Tree> ImportArchive(std::shared_ptr<const Tree> tree,const std::vector<std::uint8_t>& bytes){
    if(bytes.empty())return tree;
    std::vector<Record> verified;if(Decode(bytes,verified)!=ReadStatus::Valid)throw std::invalid_argument("Receipt archive");
    std::size_t offset=8;const auto codec=bytes[4];
    for(const auto& record:verified){const auto length=(codec==1?130:132)+113*record.effects.size();
        if(length>bytes.size()-32-offset)throw std::invalid_argument("Receipt archive span");
        tree=tree->appendRaw(codec,{bytes.data()+offset,length});offset+=length;}
    if(offset!=bytes.size()-32)throw std::invalid_argument("Receipt archive trailing data");return tree;
}
} // namespace core3d::receipt::v3
