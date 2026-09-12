#pragma once
// DRAFT: detached numeric diagnostics only, not document/edit authority.
// Caller must capture native coordinates on the owning thread and bind the
// result to document lifetime, entity ID and geometry revision before use.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace core3d::meshcheck {
using Point = std::array<double,3>;
using Triangle = std::array<Point,3>;
namespace exact {
// Sign/magnitude integers with base2^32 limbs. Every operation is exact or
// throws; capacity exhaustion must never become a clean intersection result.
// A pair of finite binary64 triangles needs <14720 bits for the highest-degree
// (degree7) interval comparison. 512 limbs also retain a margin for carries.
class Integer {
    static constexpr std::size_t maximumLimbs=512;
    std::vector<std::uint32_t> words;
    int polarity=0;
    void normalize() {
        while(!words.empty() && words.back()==0)words.pop_back();
        if(words.size()>maximumLimbs)throw std::length_error("Exact integer capacity");
        if(words.empty())polarity=0;
    }
    static int magnitudeCompare(const Integer& a,const Integer& b) {
        if(a.words.size()!=b.words.size())return a.words.size()<b.words.size()?-1:1;
        for(std::size_t i=a.words.size();i>0;--i)
            if(a.words[i-1]!=b.words[i-1])return a.words[i-1]<b.words[i-1]?-1:1;
        return 0;
    }
public:
    Integer()=default;
    static Integer shifted(std::uint64_t bits,unsigned shift,int sign=1) {
        Integer r;if(!bits)return r;
        if(shift/32+3>maximumLimbs)throw std::length_error("Exact coordinate capacity");
        r.polarity=sign;r.words.assign(shift/32+3,0);
        const unsigned rem=shift%32;const std::size_t offset=shift/32;
        const std::uint64_t lo=std::uint64_t(std::uint32_t(bits))<<rem;
        const std::uint64_t hi=(bits>>32)<<rem;
        r.words[offset]=std::uint32_t(lo);
        r.words[offset+1]=std::uint32_t((lo>>32)|hi);
        r.words[offset+2]=std::uint32_t(hi>>32);r.normalize();return r;
    }
    int sign() const{return polarity;}
    friend Integer operator-(Integer a){a.polarity=-a.polarity;return a;}
    friend Integer operator+(const Integer& a,const Integer& b) {
        if(!a.polarity)return b;if(!b.polarity)return a;
        Integer r;
        if(a.polarity==b.polarity) {
            r.polarity=a.polarity;r.words.resize(std::max(a.words.size(),b.words.size())+1);
            std::uint64_t carry=0;
            for(std::size_t i=0;i+1<r.words.size();++i) {
                const std::uint64_t sum=carry+(i<a.words.size()?a.words[i]:0ULL)+(i<b.words.size()?b.words[i]:0ULL);
                r.words[i]=std::uint32_t(sum);carry=sum>>32;
            }
            r.words.back()=std::uint32_t(carry);
        } else {
            const int order=magnitudeCompare(a,b);if(!order)return r;
            const Integer& large=order>0?a:b;const Integer& small=order>0?b:a;
            r.polarity=large.polarity;r.words.resize(large.words.size());std::uint64_t borrow=0;
            for(std::size_t i=0;i<large.words.size();++i) {
                const std::uint64_t rhs=borrow+(i<small.words.size()?small.words[i]:0ULL);
                const std::uint64_t lhs=large.words[i];r.words[i]=std::uint32_t(lhs-rhs);borrow=lhs<rhs;
            }
        }
        r.normalize();return r;
    }
    friend Integer operator-(const Integer& a,const Integer& b){return a+(-b);}
    friend Integer operator*(const Integer& a,const Integer& b) {
        Integer r;if(!a.polarity || !b.polarity)return r;
        if(a.words.size()+b.words.size()>maximumLimbs+1)throw std::length_error("Exact product capacity");
        r.polarity=a.polarity*b.polarity;r.words.assign(a.words.size()+b.words.size(),0);
        for(std::size_t i=0;i<a.words.size();++i) {
            std::uint64_t carry=0;
            for(std::size_t j=0;j<b.words.size();++j) {
                const std::uint64_t value=std::uint64_t(a.words[i])*b.words[j]+r.words[i+j]+carry;
                r.words[i+j]=std::uint32_t(value);carry=value>>32;
            }
            r.words[i+b.words.size()]=std::uint32_t(carry);
        }
        r.normalize();return r;
    }
    friend int compare(const Integer& a,const Integer& b) {
        if(a.polarity!=b.polarity)return a.polarity<b.polarity?-1:1;
        return a.polarity*magnitudeCompare(a,b);
    }
};
using Vector=std::array<Integer,3>;
using Face=std::array<Vector,3>;
inline Vector subtract(const Vector& a,const Vector& b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline Vector cross(const Vector& a,const Vector& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline Integer dot(const Vector& a,const Vector& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline bool zero(const Vector& v){return !v[0].sign()&&!v[1].sign()&&!v[2].sign();}
inline bool equal(const Vector& a,const Vector& b){return compare(a[0],b[0])==0&&compare(a[1],b[1])==0&&compare(a[2],b[2])==0;}
struct Binary {std::uint64_t mantissa;int exponent,sign;};
inline Binary decode(double value) {
    static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559,"IEEE binary64 required");
    if(!std::isfinite(value))throw std::invalid_argument("Nonfinite mesh coordinate");
    std::uint64_t bits;std::memcpy(&bits,&value,8);
    const unsigned exponent=unsigned((bits>>52)&2047);
    return {(bits&0xfffffffffffffULL)|(exponent?0x10000000000000ULL:0),
            exponent?int(exponent)-1023-52:-1074,(bits>>63)?-1:1};
}
inline std::array<Face,2> convert(const Triangle& a,const Triangle& b) {
    std::array<Binary,18> values;int base=1024;std::size_t index=0;
    for(const auto* t:{&a,&b})for(const auto& p:*t)for(double v:p) {
        auto q=decode(v);values[index++]=q;if(q.mantissa)base=std::min(base,q.exponent);
    }
    std::array<Face,2> result;index=0;
    for(auto& f:result)for(auto& p:f)for(auto& v:p) {
        const auto q=values[index++];v=q.mantissa?Integer::shifted(q.mantissa,unsigned(q.exponent-base),q.sign):Integer();
    }
    return result;
}
struct Scalar {Integer numerator,denominator;};
inline Scalar scalar(const Integer& n){return {n,Integer::shifted(1,0)};}
inline int compare(const Scalar& a,const Scalar& b){return (a.numerator*b.denominator-b.numerator*a.denominator).sign();}
struct Interval {Scalar lo,hi;bool present=false;};
inline Interval cut(const Face& f,const std::array<Integer,3>& distances,int axis) {
    Interval result;
    auto append=[&](Scalar s) {
        if(!result.present){result.lo=result.hi=std::move(s);result.present=true;return;}
        if(compare(s,result.lo)<0)result.lo=s;if(compare(s,result.hi)>0)result.hi=std::move(s);
    };
    for(int i=0;i<3;++i)if(!distances[i].sign())append(scalar(f[i][axis]));
    for(int i=0;i<3;++i) {
        const int j=(i+1)%3;if(distances[i].sign()*distances[j].sign()>=0)continue;
        Scalar s{f[i][axis]*distances[j]-f[j][axis]*distances[i],distances[j]-distances[i]};
        if(s.denominator.sign()<0){s.numerator=-s.numerator;s.denominator=-s.denominator;}
        append(std::move(s));
    }
    return result;
}
inline int orient(const Vector& a,const Vector& b,const Vector& c,int x,int y) {
    return ((b[x]-a[x])*(c[y]-a[y])-(b[y]-a[y])*(c[x]-a[x])).sign();
}
inline bool unexpected(const Triangle& first,const Triangle& second) {
    const auto pair=convert(first,second);const auto& a=pair[0];const auto& b=pair[1];
    const auto na=cross(subtract(a[1],a[0]),subtract(a[2],a[0]));
    const auto nb=cross(subtract(b[1],b[0]),subtract(b[2],b[0]));
    if(zero(na)||zero(nb))throw std::invalid_argument("Degenerate triangle");
    std::array<Integer,3> da,db;
    for(int i=0;i<3;++i){da[i]=dot(nb,subtract(a[i],b[0]));db[i]=dot(na,subtract(b[i],a[0]));}
    auto separated=[](const auto& d){return d[0].sign()!=0&&d[0].sign()==d[1].sign()&&d[0].sign()==d[2].sign();};
    if(separated(da)||separated(db))return false;
    std::vector<Vector> common;
    for(const auto& p:a)for(const auto& q:b)if(equal(p,q)){common.push_back(p);break;}
    const auto direction=cross(na,nb);
    if(!zero(direction)) {
        int axis=0;while(!direction[axis].sign())++axis;
        const auto ca=cut(a,da,axis),cb=cut(b,db,axis);if(!ca.present||!cb.present)return false;
        const auto lo=compare(ca.lo,cb.lo)>=0?ca.lo:cb.lo;
        const auto hi=compare(ca.hi,cb.hi)<=0?ca.hi:cb.hi;
        if(compare(lo,hi)>0)return false;if(common.empty())return true;
        auto lower=common[0][axis],upper=lower;
        for(const auto& p:common){if(compare(p[axis],lower)<0)lower=p[axis];if(compare(p[axis],upper)>0)upper=p[axis];}
        return compare(lo,scalar(lower))<0||compare(hi,scalar(upper))>0;
    }
    for(int i=0;i<3;++i)if(da[i].sign()||db[i].sign())return false;
    if(common.size()==3)return true;
    int drop=0;while(!na[drop].sign())++drop;const int x=(drop+1)%3,y=(drop+2)%3;
    auto inside=[&](const Vector& p,const Face& f) {
        const int d0=orient(f[0],f[1],p,x,y),d1=orient(f[1],f[2],p,x,y),d2=orient(f[2],f[0],p,x,y);
        return (d0>=0&&d1>=0&&d2>=0)||(d0<=0&&d1<=0&&d2<=0);
    };
    auto isCommon=[&](const Vector& p){return std::any_of(common.begin(),common.end(),[&](const auto& q){return equal(p,q);});};
    for(const auto& p:a)if(!isCommon(p)&&inside(p,b))return true;
    for(const auto& p:b)if(!isCommon(p)&&inside(p,a))return true;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
        const auto& u=a[i];const auto& v=a[(i+1)%3];const auto& s=b[j];const auto& t=b[(j+1)%3];
        if(orient(u,v,s,x,y)*orient(u,v,t,x,y)<0&&orient(s,t,u,x,y)*orient(s,t,v,x,y)<0)return true;
    }
    return false;
}
} // namespace exact

enum class ContactStatus {Ready,Invalid,TooLarge,Cancelled,TimedOut};
struct ContactReport {
    std::size_t triangleCount=0,candidatePairs=0;
    std::vector<std::pair<std::uint32_t,std::uint32_t>> unexpectedPairs;
};
struct ContactLimits {
    std::size_t maximumTriangles=20000,maximumPairs=2000000,maximumContacts=2048;
    std::chrono::milliseconds duration{10000};
};
inline ContactStatus AnalyzeTriangleContacts(const std::vector<Triangle>& triangles,
    ContactReport& output,const std::atomic_bool& cancelled,ContactLimits limits={}) noexcept {
    output={};
    // Validate duration before chrono arithmetic, including hostile callers.
    if(limits.maximumTriangles>20000||limits.maximumPairs>2000000||limits.maximumContacts>2048
        ||limits.duration.count()<=0||limits.duration.count()>10000)return ContactStatus::Invalid;
    const auto deadline=std::chrono::steady_clock::now()+limits.duration;
    auto checkpoint=[&] {
        if(cancelled.load(std::memory_order_relaxed))throw ContactStatus::Cancelled;
        if(std::chrono::steady_clock::now()>=deadline)throw ContactStatus::TimedOut;
    };
    try {
        checkpoint();
        if(triangles.empty())return ContactStatus::Invalid;
        if(triangles.size()>limits.maximumTriangles)return ContactStatus::TooLarge;
        struct Bounds {Point lo,hi;};std::vector<Bounds> bounds;bounds.reserve(triangles.size());
        for(const auto& t:triangles) {
            checkpoint();Bounds box{t[0],t[0]};
            for(const auto& p:t)for(int k=0;k<3;++k) {
                if(!std::isfinite(p[k])||std::abs(p[k])>1.e6)return ContactStatus::Invalid;
                box.lo[k]=std::min(box.lo[k],p[k]);box.hi[k]=std::max(box.hi[k],p[k]);
            }
            const auto f=exact::convert(t,t)[0];
            if(exact::zero(exact::cross(exact::subtract(f[1],f[0]),exact::subtract(f[2],f[0]))))return ContactStatus::Invalid;
            bounds.push_back(box);
        }
        std::vector<std::uint32_t> order(triangles.size());std::iota(order.begin(),order.end(),0);
        std::sort(order.begin(),order.end(),[&](auto a,auto b){return bounds[a].lo[0]<bounds[b].lo[0]||(bounds[a].lo[0]==bounds[b].lo[0]&&a<b);});
        std::vector<std::uint32_t> active;ContactReport report;report.triangleCount=triangles.size();
        for(auto i:order) {
            checkpoint();const auto& b=bounds[i];
            active.erase(std::remove_if(active.begin(),active.end(),[&](auto j){return bounds[j].hi[0]<b.lo[0];}),active.end());
            for(auto j:active) {
                checkpoint();const auto& a=bounds[j];
                if(b.lo[1]>a.hi[1]||a.lo[1]>b.hi[1]||b.lo[2]>a.hi[2]||a.lo[2]>b.hi[2])continue;
                if(report.candidatePairs==limits.maximumPairs)return ContactStatus::TooLarge;
                ++report.candidatePairs;
                if(exact::unexpected(triangles[i],triangles[j])) {
                    if(report.unexpectedPairs.size()==limits.maximumContacts)return ContactStatus::TooLarge;
                    report.unexpectedPairs.emplace_back(j,i);
                }
            }
            active.push_back(i);
        }
        checkpoint();output=std::move(report);return ContactStatus::Ready;
    }catch(ContactStatus status){return status;}
    catch(const std::length_error&){return ContactStatus::TooLarge;}
    catch(...){return ContactStatus::Invalid;}
}
} // namespace core3d::meshcheck
