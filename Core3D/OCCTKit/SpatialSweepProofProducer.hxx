#pragma once

// The sole K1 producer for positive curve-proof receipts.  All arithmetic used
// to extract rational Bezier spans and to decide exact joins starts from the
// persisted IEEE-754 bits.  Search bounds are outward rounded and a budget or
// depth exhaustion returns an empty result; callers cannot turn observations
// from OCCT or sampled stations into a CurveCertificate.
#include "BoundedCurveDefinition.hxx"
#include "SpatialSweepProof.hxx"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <deque>
#include <limits>
#include <stdexcept>

namespace core3d::spatial_sweep::proof_producer {

enum class Status : std::uint8_t {
    Produced = 0, InvalidInput, Cancelled, ExactArithmeticBudget,
    LeafBudget, DepthBudget, RegularityUnproved, JoinMismatch,
    NonlocalContact, NonlocalClearanceUnproved
};

struct Result {
    Status status = Status::InvalidInput;
    CurveCertificate certificate{};
    bool produced() const noexcept { return status == Status::Produced; }
};

namespace detail {
struct Budget final : std::runtime_error { Budget() : std::runtime_error("exact budget") {} };

// Fixed-capacity signed integer. Arithmetic refuses before a 4097th bit can
// be created, so the implementation enforces the profile rather than relying
// on an unbounded third-party allocator.
class cpp_int {
public:
    static constexpr std::size_t Words=MaximumExactArithmeticBits/32;
    cpp_int()=default;
    cpp_int(std::int64_t value){if(value<0){negative_=true;const std::uint64_t magnitude=std::uint64_t(-(value+1))+1;words_[0]=std::uint32_t(magnitude);words_[1]=std::uint32_t(magnitude>>32);}else{const auto magnitude=std::uint64_t(value);words_[0]=std::uint32_t(magnitude);words_[1]=std::uint32_t(magnitude>>32);}used_=2;normalize();}
    std::uint32_t bitCount()const{if(used_==0)return 1;return std::uint32_t((used_-1)*32+(32-__builtin_clz(words_[used_-1])));}
    long double toLongDouble()const{long double value=0;for(std::size_t i=used_;i>0;--i)value=value*4294967296.0L+words_[i-1];return negative_?-value:value;}
    explicit operator bool()const{return used_!=0;}
    friend bool operator==(const cpp_int&a,const cpp_int&b){if(a.used_==0&&b.used_==0)return true;if(a.negative_!=b.negative_||a.used_!=b.used_)return false;for(std::size_t i=0;i<a.used_;++i)if(a.words_[i]!=b.words_[i])return false;return true;}
    friend bool operator!=(const cpp_int&a,const cpp_int&b){return !(a==b);}
    friend bool operator<(const cpp_int&a,const cpp_int&b){if(a.negative_!=b.negative_)return a.negative_;const int cmp=compareAbs(a,b);return a.negative_?cmp>0:cmp<0;}
    friend bool operator>(const cpp_int&a,const cpp_int&b){return b<a;}
    friend bool operator<=(const cpp_int&a,const cpp_int&b){return !(b<a);}
    friend bool operator>=(const cpp_int&a,const cpp_int&b){return !(a<b);}
    cpp_int operator-()const{cpp_int out=*this;if(out.used_)out.negative_=!out.negative_;return out;}
    friend cpp_int operator+(const cpp_int&a,const cpp_int&b){if(a.negative_==b.negative_){cpp_int out=addAbs(a,b);out.negative_=a.negative_;out.normalize();return out;}const int cmp=compareAbs(a,b);if(cmp==0)return {};cpp_int out=cmp>0?subAbs(a,b):subAbs(b,a);out.negative_=cmp>0?a.negative_:b.negative_;return out;}
    friend cpp_int operator-(const cpp_int&a,const cpp_int&b){return a+(-b);}
    friend cpp_int operator*(const cpp_int&a,const cpp_int&b){cpp_int out;if(!a.used_||!b.used_)return out;if(a.used_+b.used_>Words+1)throw Budget();for(std::size_t i=0;i<a.used_;++i){std::uint64_t carry=0;for(std::size_t j=0;j<b.used_;++j){if(i+j>=Words)throw Budget();const std::uint64_t value=std::uint64_t(a.words_[i])*b.words_[j]+out.words_[i+j]+carry;out.words_[i+j]=std::uint32_t(value);carry=value>>32;}std::size_t k=i+b.used_;while(carry){if(k>=Words)throw Budget();const std::uint64_t value=std::uint64_t(out.words_[k])+carry;out.words_[k]=std::uint32_t(value);carry=value>>32;++k;}}out.used_=std::min(Words,a.used_+b.used_);out.negative_=a.negative_!=b.negative_;out.normalize();return out;}
    friend cpp_int operator/(const cpp_int&a,const cpp_int&b){cpp_int q,r;divmod(a,b,q,r);return q;}
    friend cpp_int operator%(const cpp_int&a,const cpp_int&b){cpp_int q,r;divmod(a,b,q,r);return r;}
    cpp_int& operator+=(const cpp_int&v){return *this=*this+v;}
    cpp_int& operator-=(const cpp_int&v){return *this=*this-v;}
    cpp_int& operator*=(const cpp_int&v){return *this=*this*v;}
    cpp_int& operator/=(const cpp_int&v){return *this=*this/v;}
    // Refuses only when the shifted value would genuinely create a 4097th
    // bit; an exactly 4096-bit result (the profile boundary) is admitted.
    cpp_int& operator<<=(unsigned shift){if(!used_||shift==0)return *this;const unsigned whole=shift/32,bits=shift%32;if(used_+whole>Words)throw Budget();if(bits&&used_+whole>=Words&&(words_[used_-1]>>(32-bits)))throw Budget();if(whole){for(std::size_t i=used_;i>0;--i)words_[i-1+whole]=words_[i-1];for(unsigned i=0;i<whole;++i)words_[i]=0;used_+=whole;}if(bits){std::uint64_t carry=0;for(std::size_t i=0;i<used_;++i){const std::uint64_t value=(std::uint64_t(words_[i])<<bits)|carry;words_[i]=std::uint32_t(value);carry=value>>32;}if(carry){if(used_>=Words)throw Budget();words_[used_++]=std::uint32_t(carry);}}return *this;}
private:
    static int compareAbs(const cpp_int&a,const cpp_int&b){if(a.used_!=b.used_)return a.used_<b.used_?-1:1;for(std::size_t i=a.used_;i>0;--i)if(a.words_[i-1]!=b.words_[i-1])return a.words_[i-1]<b.words_[i-1]?-1:1;return 0;}
    // Each limb is widened to 64 bits BEFORE the first addition, so the carry
    // out of a 0xffffffff+0xffffffff limb pair is retained instead of being
    // wrapped away; a carry out of the most significant used limb counts
    // against the 4096-bit profile and refuses instead of overflowing.
    static cpp_int addAbs(const cpp_int&a,const cpp_int&b){cpp_int out;const std::size_t n=std::max(a.used_,b.used_);std::uint64_t carry=0;for(std::size_t i=0;i<n;++i){const std::uint64_t value=std::uint64_t(i<a.used_?a.words_[i]:0)+std::uint64_t(i<b.used_?b.words_[i]:0)+carry;out.words_[i]=std::uint32_t(value);carry=value>>32;}out.used_=n;if(carry){if(n>=Words)throw Budget();out.words_[out.used_++]=std::uint32_t(carry);}return out;}
    static cpp_int subAbs(const cpp_int&a,const cpp_int&b){cpp_int out;std::uint64_t borrow=0;for(std::size_t i=0;i<a.used_;++i){const std::uint64_t av=a.words_[i],bv=(i<b.used_?b.words_[i]:0)+borrow;out.words_[i]=std::uint32_t(av-bv);borrow=av<bv;}out.used_=a.used_;out.normalize();return out;}
    static void setBit(cpp_int&v,std::uint32_t bit){const std::size_t word=bit/32;if(word>=Words)throw Budget();v.words_[word]|=std::uint32_t(1)<<(bit%32);v.used_=std::max(v.used_,word+1);}
    static void divmod(const cpp_int&a,const cpp_int&b,cpp_int&q,cpp_int&r){if(!b.used_)throw Budget();cpp_int dividend=a;dividend.negative_=false;cpp_int divisor=b;divisor.negative_=false;if(compareAbs(dividend,divisor)<0){r=dividend;r.negative_=a.negative_;return;}const std::uint32_t bits=dividend.bitCount();for(std::uint32_t bit=bits;bit>0;--bit){r<<=1;if((dividend.words_[(bit-1)/32]>>((bit-1)%32))&1)setBit(r,0);r.normalize();if(compareAbs(r,divisor)>=0){r=subAbs(r,divisor);setBit(q,bit-1);}}q.negative_=a.negative_!=b.negative_;r.negative_=a.negative_;q.normalize();r.normalize();}
    void normalize(){while(used_&&words_[used_-1]==0)--used_;if(!used_)negative_=false;}
    std::array<std::uint32_t,Words> words_{};std::size_t used_=0;bool negative_=false;
};

inline cpp_int Abs(cpp_int value) { return value < 0 ? -value : value; }
inline cpp_int Gcd(cpp_int a, cpp_int b) {
    a = Abs(a); b = Abs(b);
    while (b != 0) { cpp_int r = a % b; a = b; b = r; }
    return a == 0 ? cpp_int(1) : a;
}
inline std::uint32_t Bits(const cpp_int& value) { return value.bitCount(); }

class Rational {
public:
    Rational() = default;
    explicit Rational(std::int64_t value) : n_(value) {}
    Rational(cpp_int numerator, cpp_int denominator) : n_(std::move(numerator)), d_(std::move(denominator)) {
        normalize();
    }
    static Rational FromDouble(double value) {
        if (!std::isfinite(value)) throw Budget();
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits));
        const bool negative = (bits >> 63) != 0;
        const std::uint64_t rawExponent = (bits >> 52) & 0x7ff;
        std::uint64_t mantissa = bits & ((std::uint64_t(1) << 52) - 1);
        if (rawExponent == 0 && mantissa == 0) return Rational{};
        int exponent = 0;
        if (rawExponent == 0) exponent = -1022 - 52;
        else { mantissa |= std::uint64_t(1) << 52; exponent = int(rawExponent) - 1023 - 52; }
        cpp_int numerator = mantissa, denominator = 1;
        if (exponent >= 0) numerator <<= exponent; else denominator <<= -exponent;
        if (negative) numerator = -numerator;
        return Rational(std::move(numerator), std::move(denominator));
    }
    bool zero() const noexcept { return n_ == 0; }
    int sign() const noexcept { return n_ == 0 ? 0 : n_ < 0 ? -1 : 1; }
    const cpp_int& numerator() const noexcept { return n_; }
    const cpp_int& denominator() const noexcept { return d_; }
    std::uint32_t bits() const { return std::max(Bits(n_), Bits(d_)); }
    long double value() const { return n_.toLongDouble() / d_.toLongDouble(); }
    friend bool operator==(const Rational& a, const Rational& b) { return a.n_ == b.n_ && a.d_ == b.d_; }
    friend bool operator<(const Rational& a, const Rational& b) { return a.n_ * b.d_ < b.n_ * a.d_; }
    friend Rational operator-(const Rational& a) { return Rational(-a.n_, a.d_); }
    friend Rational operator+(const Rational& a, const Rational& b) { return Rational(a.n_*b.d_ + b.n_*a.d_, a.d_*b.d_); }
    friend Rational operator-(const Rational& a, const Rational& b) { return a + (-b); }
    friend Rational operator*(const Rational& a, const Rational& b) { return Rational(a.n_*b.n_, a.d_*b.d_); }
    friend Rational operator/(const Rational& a, const Rational& b) {
        if (b.n_ == 0) throw Budget();
        return Rational(a.n_*b.d_, a.d_*b.n_);
    }
private:
    void normalize() {
        if (d_ == 0) throw Budget();
        if (d_ < 0) { d_ = -d_; n_ = -n_; }
        const cpp_int divisor = Gcd(n_, d_); n_ /= divisor; d_ /= divisor;
        if (bits() > MaximumExactArithmeticBits) throw Budget();
    }
    cpp_int n_ = 0, d_ = 1;
};

using R3 = std::array<Rational, 3>;
using H4 = std::array<Rational, 4>;
using Poly = std::vector<Rational>;

inline Rational Choose(unsigned n, unsigned k) {
    if (k > n) return Rational{};
    k = std::min(k, n-k); cpp_int value = 1;
    for (unsigned i=1;i<=k;++i) { value *= n-k+i; value /= i; }
    return Rational(std::move(value), 1);
}
inline Poly Add(const Poly& a, const Poly& b) {
    if (a.size() != b.size()) throw Budget();
    Poly out(a.size()); for (std::size_t i=0;i<a.size();++i) out[i]=a[i]+b[i]; return out;
}
inline Poly Subtract(const Poly& a, const Poly& b) {
    if (a.size() != b.size()) throw Budget();
    Poly out(a.size()); for (std::size_t i=0;i<a.size();++i) out[i]=a[i]-b[i]; return out;
}
inline Poly Scale(const Poly& a, const Rational& b) {
    Poly out(a.size()); for (std::size_t i=0;i<a.size();++i) out[i]=a[i]*b; return out;
}
inline Poly Derivative(const Poly& a) {
    if (a.size() < 2) return {Rational{}};
    const Rational degree(std::int64_t(a.size()-1)); Poly out(a.size()-1);
    for (std::size_t i=0;i<out.size();++i) out[i]=(a[i+1]-a[i])*degree;
    return out;
}
inline Poly Product(const Poly& a, const Poly& b) {
    const unsigned p = unsigned(a.size()-1), q = unsigned(b.size()-1);
    Poly out(p+q+1);
    for (unsigned k=0;k<=p+q;++k) {
        Rational sum;
        const unsigned lo = k>q ? k-q : 0, hi = std::min(p,k);
        for (unsigned i=lo;i<=hi;++i)
            sum = sum + a[i]*b[k-i]*Choose(p,i)*Choose(q,k-i)/Choose(p+q,k);
        out[k]=sum;
    }
    return out;
}
inline std::pair<Poly,Poly> Split(const Poly& p) {
    std::vector<Poly> rows; rows.push_back(p);
    while (rows.back().size()>1) {
        Poly next(rows.back().size()-1);
        for (std::size_t i=0;i<next.size();++i) next[i]=(rows.back()[i]+rows.back()[i+1])/Rational(2);
        rows.push_back(std::move(next));
    }
    Poly left(p.size()),right(p.size());
    for (std::size_t i=0;i<p.size();++i) { left[i]=rows[i][0]; right[p.size()-1-i]=rows[i].back(); }
    return {std::move(left),std::move(right)};
}
// Round `value` `steps` ulps in one direction. Used to make the side of every
// composite round-to-nearest expression below certain instead of assumed.
inline double Above(double value,unsigned steps){for(unsigned i=0;i<steps;++i){const double next=std::nextafter(value,std::numeric_limits<double>::infinity());if(next==value)break;value=next;}return value;}
inline double Below(double value,unsigned steps){for(unsigned i=0;i<steps;++i){const double next=std::nextafter(value,-std::numeric_limits<double>::infinity());if(next==value)break;value=next;}return value;}
// Outward binary64 bounds of an exact rational. Rational::value() divides two
// rounded conversions (long double is binary64 on this arm64 host), so a
// single nextafter around the quotient cannot guarantee an enclosure for all
// admitted 4096-bit operands. Instead the candidate's exact persisted bits
// are compared against the rational and stepped until the exact comparison
// proves the claimed side; nonfinite or unrepresentable results refuse
// through the exact-arithmetic budget rather than emitting a false bound.
inline double Down(const Rational& r) {
    double out=double(r.value()); if(!std::isfinite(out)) throw Budget();
    while(r<Rational::FromDouble(out)){out=std::nextafter(out,-std::numeric_limits<double>::infinity());if(!std::isfinite(out))throw Budget();}
    return out;
}
inline double Up(const Rational& r) {
    double out=double(r.value()); if(!std::isfinite(out)) throw Budget();
    while(Rational::FromDouble(out)<r){out=std::nextafter(out,std::numeric_limits<double>::infinity());if(!std::isfinite(out))throw Budget();}
    return out;
}
// Directed non-negative binary64 operations.  Every elementary rounded
// operation is widened before its result is reused, rather than assuming a
// fixed number of ulps is enough for a composite expression.
inline double AddDown(double a,double b){return Below(a+b,1);}
inline double AddUp(double a,double b){return Above(a+b,1);}
inline double MulDown(double a,double b){return Below(a*b,1);}
inline double MulUp(double a,double b){return Above(a*b,1);}
inline double DivDown(double a,double b){if(!(b>0))throw Budget();return Below(a/b,1);}
inline double DivUp(double a,double b){if(!(b>0))throw Budget();return Above(a/b,1);}
inline double SqrtDown(double a){return a<=0?0:Below(std::sqrt(a),1);}
inline double SqrtUp(double a){if(a<0||!std::isfinite(a))throw Budget();return Above(std::sqrt(a),1);}
inline Interval Hull(const Poly& p) {
    auto lo=std::min_element(p.begin(),p.end()),hi=std::max_element(p.begin(),p.end());
    return {Down(*lo),Up(*hi)};
}
inline double MaximumAbs(const Interval& x) { return std::max(std::abs(x.lower),std::abs(x.upper)); }
inline double SquareLower(const Interval& x) {
    if (x.lower<=0&&x.upper>=0) return 0; return std::min(x.lower*x.lower,x.upper*x.upper);
}
inline double SquareUpper(const Interval& x) { const double m=MaximumAbs(x); return MulUp(m,m); }

struct Span {
    std::array<Poly,4> h;
    Rational u0,u1;
    std::uint32_t depth=0;
};
inline std::pair<Span,Span> Split(const Span& s) {
    Span a=s,b=s; a.u1=(s.u0+s.u1)/Rational(2); b.u0=a.u1; ++a.depth;++b.depth;
    for(int c=0;c<4;++c) { auto halves=Split(s.h[c]); a.h[c]=std::move(halves.first); b.h[c]=std::move(halves.second); }
    return {std::move(a),std::move(b)};
}

inline H4 Blend(const H4& a,const H4& b,const Rational& alpha) {
    H4 out; for(int c=0;c<4;++c) out[c]=a[c]*(Rational(1)-alpha)+b[c]*alpha; return out;
}
inline bool InsertOnce(unsigned degree,const Rational& value,std::vector<Rational>& knots,
                       std::vector<H4>& poles) {
    const int n=int(poles.size())-1,m=n+int(degree)+1;
    int k=-1,s=0; for(int i=0;i<=m;++i) { if (!(value<knots[i])) k=i; if(knots[i]==value)++s; }
    if(k<0||s>=int(degree)) return false;
    std::vector<Rational> uq(knots.size()+1); std::vector<H4> q(poles.size()+1);
    for(int i=0;i<=k;++i) uq[i]=knots[i]; uq[k+1]=value; for(int i=k+1;i<=m;++i) uq[i+1]=knots[i];
    for(int i=0;i<=k-int(degree);++i) q[i]=poles[i];
    for(int i=k-s;i<=n;++i) q[i+1]=poles[i];
    for(int i=k-int(degree)+1;i<=k-s;++i) {
        const Rational alpha=(value-knots[i])/(knots[i+degree]-knots[i]); q[i]=Blend(poles[i-1],poles[i],alpha);
    }
    knots=std::move(uq); poles=std::move(q); return true;
}

inline R3 FramePoint(const bounded_curve::Definition& curve,
                     const bounded_curve::ControlPoint& point,double mmPerUnit) {
    R3 out; const Rational scale=Rational::FromDouble(mmPerUnit);
    for(int axis=0;axis<3;++axis) {
        Rational value=Rational::FromDouble(curve.frame.origin[axis]);
        value=value+Rational::FromDouble(curve.frame.xAxis[axis])*Rational::FromDouble(point.local[0]);
        value=value+Rational::FromDouble(curve.frame.yAxis[axis])*Rational::FromDouble(point.local[1]);
        value=value+Rational::FromDouble(curve.frame.zAxis[axis])*Rational::FromDouble(point.local[2]);
        out[axis]=value*scale;
    }
    return out;
}

inline bool Extract(const bounded_curve::Definition& curve,double mmPerUnit,std::vector<Span>& spans) {
    if(bounded_curve::Validate(curve)!=bounded_curve::Refusal::None||curve.domain!=bounded_curve::Domain::Path3D
       ||!std::isfinite(mmPerUnit)||mmPerUnit<=0) return false;
    std::vector<Rational> knots; for(const auto& k:curve.knots) for(unsigned j=0;j<k.multiplicity;++j) knots.push_back(Rational::FromDouble(k.value));
    std::vector<H4> poles; poles.reserve(curve.controlPoints.size());
    for(std::size_t i=0;i<curve.controlPoints.size();++i) {
        const Rational w=Rational::FromDouble(curve.weights.empty()?1:curve.weights[i]);
        const R3 p=FramePoint(curve,curve.controlPoints[i],mmPerUnit); poles.push_back({p[0]*w,p[1]*w,p[2]*w,w});
    }
    for(std::size_t distinct=1;distinct+1<curve.knots.size();++distinct) {
        const Rational u=Rational::FromDouble(curve.knots[distinct].value);
        for(unsigned j=curve.knots[distinct].multiplicity;j<curve.degree;++j) if(!InsertOnce(curve.degree,u,knots,poles)) return false;
    }
    const std::size_t count=(poles.size()-1)/curve.degree; if(count==0||count*curve.degree+1!=poles.size()||count+1!=curve.knots.size()) return false;
    spans.clear();spans.reserve(count);
    for(std::size_t s=0;s<count;++s) {
        Span span; span.u0=Rational::FromDouble(curve.knots[s].value);
        span.u1=Rational::FromDouble(curve.knots[s+1].value);
        for(int c=0;c<4;++c) { span.h[c].resize(curve.degree+1); for(unsigned i=0;i<=curve.degree;++i) span.h[c][i]=poles[s*curve.degree+i][c]; }
        spans.push_back(std::move(span));
    }
    return true;
}

struct Bounds {
    double speedLower=0,speedUpper=0,curvatureUpper=0,lengthLower=0,lengthUpper=0,maxAbs=0;
    std::array<Interval,3> position{};
};
// If n(t) is the Bernstein numerator of the rational derivative, convexity
// of the norm and integral(B_i)=1/count give
//   integral |n(t)| dt <= average_i |n_i|.
// Dividing by the certified lower bound of w(t)^2 therefore bounds arc length
// without replacing the exact proof by sampling or a supremum rectangle.
inline double BernsteinLengthUpper(const std::array<Poly,3>& n,double weightSquareLower) {
    if(n[0].empty()||n[1].size()!=n[0].size()||n[2].size()!=n[0].size())throw Budget();
    double total=0;
    for(std::size_t i=0;i<n[0].size();++i){Rational squared;for(int c=0;c<3;++c)squared=squared+n[c][i]*n[c][i];total=AddUp(total,SqrtUp(std::max(0.0,Up(squared))));}
    return DivUp(DivUp(total,double(n[0].size())),weightSquareLower);
}
// Positive rational Bernstein functions form a normalized totally-positive
// basis.  The variation-diminishing property bounds the curve's total
// variation, and hence its arc length, by its Euclidean control polygon.
inline double RationalControlPolygonUpper(const Span& s){double total=0;for(std::size_t i=1;i<s.h[0].size();++i){Rational squared;for(int c=0;c<3;++c){const Rational d=s.h[c][i]/s.h[3][i]-s.h[c][i-1]/s.h[3][i-1];squared=squared+d*d;}total=AddUp(total,SqrtUp(std::max(0.0,Up(squared))));}return total;}
inline std::uint32_t LengthSubdivisionDepth(const Span& s){return std::min<std::uint32_t>(2,MaximumBisectionDepth-s.depth);}
inline std::pair<double,double> LengthBounds(const Span& s,std::uint32_t depth) {
    if(depth){auto halves=Split(s);const auto a=LengthBounds(halves.first,depth-1),b=LengthBounds(halves.second,depth-1);return {AddDown(a.first,b.first),AddUp(a.second,b.second)};}
    const Poly& w=s.h[3];const Interval wi=Hull(w);if(wi.lower<=0)throw Budget();const Poly wd=Derivative(w);std::array<Poly,3> n;for(int c=0;c<3;++c)n[c]=Subtract(Product(Derivative(s.h[c]),w),Product(s.h[c],wd));
    const double upper=std::min(BernsteinLengthUpper(n,MulDown(wi.lower,wi.lower)),RationalControlPolygonUpper(s));Rational squared;for(int c=0;c<3;++c){const Rational d=s.h[c].back()/w.back()-s.h[c].front()/w.front();squared=squared+d*d;}return {SqrtDown(std::max(0.0,Down(squared))),upper};
}
inline Bounds Bound(const Span& s,const Rational& wholeRange) {
    Bounds out; const Poly& w=s.h[3]; const Interval wi=Hull(w); if(wi.lower<=0) throw Budget();
    std::array<Poly,3> n,a;
    const Poly wd=Derivative(w);
    for(int c=0;c<3;++c) { n[c]=Subtract(Product(Derivative(s.h[c]),w),Product(s.h[c],wd)); a[c]=Subtract(Product(Derivative(n[c]),w),Scale(Product(n[c],wd),Rational(2))); }
    Poly norm2=Add(Add(Product(n[0],n[0]),Product(n[1],n[1])),Product(n[2],n[2])); const Interval ni=Hull(norm2);
    const double nlo=SqrtDown(std::max(0.0,ni.lower)),nhi=SqrtUp(std::max(0.0,ni.upper));
    const Rational factor=wholeRange/(s.u1-s.u0); const double factorLo=Down(factor),factorHi=Up(factor);
    const double weightSquareUpper=MulUp(wi.upper,wi.upper),weightSquareLower=MulDown(wi.lower,wi.lower);
    out.speedLower=MulDown(DivDown(nlo,weightSquareUpper),factorLo);
    out.speedUpper=MulUp(DivUp(nhi,weightSquareLower),factorHi);
    const auto length=LengthBounds(s,LengthSubdivisionDepth(s));out.lengthLower=length.first;out.lengthUpper=std::min(DivUp(nhi,weightSquareLower),length.second);
    for(int c=0;c<3;++c) {
        Poly euclidean(s.h[c].size());for(std::size_t i=0;i<euclidean.size();++i)euclidean[i]=s.h[c][i]/w[i];
        out.position[c]=Hull(euclidean);
        out.maxAbs=std::max(out.maxAbs,MaximumAbs(out.position[c]));
    }
    std::array<Interval,3> cross{};
    for(int c=0;c<3;++c){const int i=(c+1)%3,j=(c+2)%3;cross[c]=Hull(Subtract(Product(n[i],a[j]),Product(n[j],a[i])));}
    const double crossSquared=AddUp(AddUp(SquareUpper(cross[0]),SquareUpper(cross[1])),SquareUpper(cross[2]));
    const double crossUpper=SqrtUp(crossSquared);
    const double nloCubed=MulDown(MulDown(nlo,nlo),nlo);
    out.curvatureUpper=nlo>0&&nloCubed>0?DivUp(MulUp(wi.upper,crossUpper),nloCubed):std::numeric_limits<double>::infinity();
    return out;
}
inline double BoxDistance(const Bounds& a,const Bounds& b) {
    double squared=0;for(int c=0;c<3;++c){double gap=0;if(a.position[c].upper<b.position[c].lower)gap=std::max(0.0,Below(b.position[c].lower-a.position[c].upper,1));else if(b.position[c].upper<a.position[c].lower)gap=std::max(0.0,Below(a.position[c].lower-b.position[c].upper,1));squared=AddDown(squared,MulDown(gap,gap));}return SqrtDown(squared);
}

inline std::array<Rational,3> Endpoint(const Span& s,bool end){const std::size_t i=end?s.h[0].size()-1:0;std::array<Rational,3> out;for(int c=0;c<3;++c)out[c]=s.h[c][i]/s.h[3][i];return out;}
// Contact witness. Every span endpoint lies on its segment, so the smallest
// endpoint-pair distance (each coordinate exact, the accumulation rounded
// strictly upward) is a genuine UPPER enclosure of the minimum centreline
// distance between the two segments. BoxDistance is only a lower bound and
// can never by itself prove contact; without an upper witness the pair must
// stay on the split/unresolved path.
inline double ContactWitness(const Span& a,const Span& b) {
    double best=std::numeric_limits<double>::infinity();
    for(int ae=0;ae<2;++ae)for(int be=0;be<2;++be){const auto pa=Endpoint(a,ae!=0),pb=Endpoint(b,be!=0);Rational squared;for(int c=0;c<3;++c){const Rational d=pa[c]-pb[c];squared=squared+d*d;}best=std::min(best,SqrtUp(std::max(0.0,Up(squared))));}
    return best;
}

inline R3 Velocity(const Span& s,bool end) {
    R3 out;const auto index=end?s.h[0].size()-1:0;const auto nIndex=end?s.h[0].size()-2:0;
    const Rational degree(std::int64_t(s.h[0].size()-1)),du=s.u1-s.u0,w=s.h[3][index];
    const Rational wd=(end?s.h[3][index]-s.h[3][nIndex]:s.h[3][nIndex+1]-s.h[3][index])*degree/du;
    for(int c=0;c<3;++c){const Rational xd=(end?s.h[c][index]-s.h[c][nIndex]:s.h[c][nIndex+1]-s.h[c][index])*degree/du;out[c]=(xd*w-s.h[c][index]*wd)/(w*w);}return out;
}
inline R3 Acceleration(const Span& s,bool end) {
    const std::size_t p=s.h[0].size()-1;if(p<2)return {};
    const std::size_t index=end?p:0,i1=end?p-1:1,i2=end?p-2:2;const Rational degree{std::int64_t(p)},du=s.u1-s.u0,w=s.h[3][index];
    R3 out;const Rational wd=(end?s.h[3][index]-s.h[3][i1]:s.h[3][i1]-s.h[3][index])*degree/du;
    const Rational wdd=(end?s.h[3][index]-s.h[3][i1]*Rational(2)+s.h[3][i2]:s.h[3][i2]-s.h[3][i1]*Rational(2)+s.h[3][index])*degree*Rational(std::int64_t(p-1))/(du*du);
    for(int c=0;c<3;++c){const Rational xd=(end?s.h[c][index]-s.h[c][i1]:s.h[c][i1]-s.h[c][index])*degree/du;const Rational xdd=(end?s.h[c][index]-s.h[c][i1]*Rational(2)+s.h[c][i2]:s.h[c][i2]-s.h[c][i1]*Rational(2)+s.h[c][index])*degree*Rational(std::int64_t(p-1))/(du*du);const Rational n=xd*w-s.h[c][index]*wd;out[c]=(xdd*w-s.h[c][index]*wdd)/(w*w)-n*Rational(2)*wd/(w*w*w);}return out;
}
inline bool ExactG2(const Span& left,const Span& right) {
    const R3 v=Velocity(left,true),w=Velocity(right,false),a=Acceleration(left,true),b=Acceleration(right,false);
    R3 cross{{v[1]*w[2]-v[2]*w[1],v[2]*w[0]-v[0]*w[2],v[0]*w[1]-v[1]*w[0]}};if(!cross[0].zero()||!cross[1].zero()||!cross[2].zero())return false;
    Rational dot;for(int c=0;c<3;++c)dot=dot+v[c]*w[c];if(dot.sign()<=0)return false;
    Rational vv,ww,va,wb;for(int c=0;c<3;++c){vv=vv+v[c]*v[c];ww=ww+w[c]*w[c];va=va+v[c]*a[c];wb=wb+w[c]*b[c];}
    for(int c=0;c<3;++c)if(!(a[c]/vv-v[c]*va/(vv*vv)==b[c]/ww-w[c]*wb/(ww*ww)))return false;return true;
}
inline bool SameEndpoint(const Span& a,bool ae,const Span& b,bool be) {
    const std::size_t ai=ae?a.h[0].size()-1:0,bi=be?b.h[0].size()-1:0;for(int c=0;c<3;++c)if(!(a.h[c][ai]/a.h[3][ai]==b.h[c][bi]/b.h[3][bi]))return false;return true;
}
} // namespace detail

inline Result Produce(const bounded_curve::Definition& curve,double millimetresPerNativeUnit,
                      double radiusEnvelopeMM,bool closed,Interval holonomyRadians={0,0},
                      const std::atomic_bool* cancelled=nullptr) noexcept {
    Result result; try {
        if(!std::isfinite(radiusEnvelopeMM)||radiusEnvelopeMM<=0||!Valid(holonomyRadians))return result;
        std::vector<detail::Span> leaves;if(!detail::Extract(curve,millimetresPerNativeUnit,leaves))return result;
        const detail::Rational whole=detail::Rational::FromDouble(curve.knots.back().value)-detail::Rational::FromDouble(curve.knots.front().value);
        const auto original=leaves;
        for(std::size_t i=0;i+1<original.size();++i)if(!detail::SameEndpoint(original[i],true,original[i+1],false)||!detail::ExactG2(original[i],original[i+1])){result.status=Status::JoinMismatch;return result;}
        const bool seamSame=detail::SameEndpoint(original.back(),true,original.front(),false),seamG2=seamSame&&detail::ExactG2(original.back(),original.front());
        // The fixed two-level enclosure evaluator does not create retained
        // proof leaves, but its deepest de Casteljau level is reported in the
        // unchanged depth budget.  MaximumProofLeaves continues to bound the
        // retained adaptive partition used by every downstream certificate.
        std::uint32_t maximumDepth=detail::LengthSubdivisionDepth(leaves.front());std::vector<detail::Bounds> acceptedBounds;acceptedBounds.reserve(MaximumProofLeaves);
        for(std::size_t index=0;index<leaves.size();) {
            if(cancelled&&cancelled->load()){result.status=Status::Cancelled;return result;}
            const auto bounds=detail::Bound(leaves[index],whole);const bool split=bounds.speedLower<=64*PositionalEpsilonMM||bounds.lengthUpper-bounds.lengthLower>1e-7*std::max(1.0,bounds.lengthUpper)||bounds.lengthUpper>2*radiusEnvelopeMM;
            if(!split){acceptedBounds.push_back(bounds);++index;continue;}if(leaves[index].depth>=MaximumBisectionDepth){result.status=bounds.speedLower<=64*PositionalEpsilonMM?Status::RegularityUnproved:Status::DepthBudget;return result;}if(leaves.size()>=MaximumProofLeaves){result.status=Status::LeafBudget;return result;}
            auto halves=detail::Split(leaves[index]);maximumDepth=std::max(maximumDepth,halves.first.depth+detail::LengthSubdivisionDepth(halves.first));leaves[index]=std::move(halves.first);leaves.insert(leaves.begin()+index+1,std::move(halves.second));
        }
        if(acceptedBounds.size()!=leaves.size())throw detail::Budget();std::vector<detail::Bounds> bounds=std::move(acceptedBounds);double lengthLo=0,lengthHi=0,speedLo=std::numeric_limits<double>::infinity(),speedHi=0,curvature=0,extent=0;
        for(const auto& b:bounds){lengthLo=detail::AddDown(lengthLo,b.lengthLower);lengthHi=detail::AddUp(lengthHi,b.lengthUpper);speedLo=std::min(speedLo,b.speedLower);speedHi=std::max(speedHi,b.speedUpper);curvature=std::max(curvature,b.curvatureUpper);extent=std::max(extent,b.maxAbs);}
        std::vector<double> prefixLo(leaves.size()+1),prefixHi(leaves.size()+1);for(std::size_t i=0;i<leaves.size();++i){prefixLo[i+1]=detail::AddDown(prefixLo[i],bounds[i].lengthLower);prefixHi[i+1]=detail::AddUp(prefixHi[i],bounds[i].lengthUpper);}
        double nonlocalLower=std::numeric_limits<double>::max(),nonlocalUpper=std::numeric_limits<double>::max();std::size_t pairs=0;bool anyNonlocal=false;
        struct PairTask{detail::Span a,b;detail::Bounds ab,bb;double aStartLo=0,aStartHi=0,bStartLo=0,bStartHi=0;std::uint32_t depth=0;};
        struct RangeTask{std::size_t a0=0,a1=0,b0=0,b1=0;detail::Bounds ab,bb;};
        auto rangeBounds=[&](std::size_t first,std::size_t last){detail::Bounds out=bounds[first];out.lengthLower=out.lengthUpper=0;for(int c=0;c<3;++c)out.position[c]=bounds[first].position[c];for(std::size_t i=first;i<last;++i){out.lengthLower=detail::AddDown(out.lengthLower,bounds[i].lengthLower);out.lengthUpper=detail::AddUp(out.lengthUpper,bounds[i].lengthUpper);for(int c=0;c<3;++c){out.position[c].lower=std::min(out.position[c].lower,bounds[i].position[c].lower);out.position[c].upper=std::max(out.position[c].upper,bounds[i].position[c].upper);}}return out;};
        auto separation=[&](std::size_t a0,std::size_t a1,std::size_t b0,std::size_t b1,double& lo,double& hi){lo=std::max(0.0,detail::Below(prefixLo[b0]-prefixHi[a1],1));hi=std::max(0.0,detail::Above(prefixHi[b1]-prefixLo[a0],1));if(closed){const double wrapLo=std::max(0.0,detail::Below(lengthLo-hi,1)),wrapHi=std::max(0.0,detail::Above(lengthHi-lo,1));if(wrapHi<hi){lo=wrapLo;hi=wrapHi;}}};
        // Traverse a bounded binary hierarchy over contiguous proof leaves.
        // One clear range pair discharges every Cartesian leaf pair beneath it,
        // avoiding the quadratic eager queue while retaining complete domain
        // coverage.  A self range splits into left/left, left/right and
        // right/right; distinct ranges split only their larger side.
        constexpr std::size_t PendingLimit=2*MaximumBisectionDepth+3;
        std::vector<RangeTask> ranges;ranges.reserve(PendingLimit);ranges.push_back({0,leaves.size(),0,leaves.size(),rangeBounds(0,leaves.size()),rangeBounds(0,leaves.size())});
        std::vector<PairTask> pending;pending.reserve(PendingLimit);
        while(!ranges.empty()){
            if(cancelled&&cancelled->load()){result.status=Status::Cancelled;return result;}if(++pairs>MaximumCurvePairCells){result.status=Status::NonlocalClearanceUnproved;return result;}RangeTask range=std::move(ranges.back());ranges.pop_back();
            const bool same=range.a0==range.b0&&range.a1==range.b1;const std::size_t an=range.a1-range.a0,bn=range.b1-range.b0;if(same&&an==1)continue;
            double sepLo=0,sepHi=0;separation(range.a0,range.a1,range.b0,range.b1,sepLo,sepHi);if(sepHi<=4*radiusEnvelopeMM)continue;const double rangeDistance=detail::BoxDistance(range.ab,range.bb);if(rangeDistance>2*radiusEnvelopeMM+16*PositionalEpsilonMM){anyNonlocal=true;nonlocalLower=std::min(nonlocalLower,rangeDistance);continue;}
            if(same){const std::size_t mid=range.a0+an/2;const auto left=rangeBounds(range.a0,mid),right=rangeBounds(mid,range.a1);if(ranges.size()+3>PendingLimit){result.status=Status::NonlocalClearanceUnproved;return result;}ranges.push_back({mid,range.a1,mid,range.a1,right,right});ranges.push_back({range.a0,mid,mid,range.a1,left,right});ranges.push_back({range.a0,mid,range.a0,mid,left,left});continue;}
            if(an>1||bn>1){if(ranges.size()+2>PendingLimit){result.status=Status::NonlocalClearanceUnproved;return result;}if(an>=bn&&an>1){const std::size_t mid=range.a0+an/2;const auto left=rangeBounds(range.a0,mid),right=rangeBounds(mid,range.a1);ranges.push_back({mid,range.a1,range.b0,range.b1,right,range.bb});ranges.push_back({range.a0,mid,range.b0,range.b1,left,range.bb});}else{const std::size_t mid=range.b0+bn/2;const auto left=rangeBounds(range.b0,mid),right=rangeBounds(mid,range.b1);ranges.push_back({range.a0,range.a1,mid,range.b1,range.ab,right});ranges.push_back({range.a0,range.a1,range.b0,mid,range.ab,left});}continue;}
            pending.clear();pending.push_back({leaves[range.a0],leaves[range.b0],bounds[range.a0],bounds[range.b0],prefixLo[range.a0],prefixHi[range.a0],prefixLo[range.b0],prefixHi[range.b0],0});
            while(!pending.empty()){
                if(cancelled&&cancelled->load()){result.status=Status::Cancelled;return result;}if(++pairs>MaximumCurvePairCells){result.status=Status::NonlocalClearanceUnproved;return result;}PairTask task=std::move(pending.back());pending.pop_back();
                const double aEndHi=detail::AddUp(task.aStartHi,task.ab.lengthUpper),bEndHi=detail::AddUp(task.bStartHi,task.bb.lengthUpper);double taskSepLo=std::max(0.0,detail::Below(task.bStartLo-aEndHi,1)),taskSepHi=std::max(0.0,detail::Above(bEndHi-task.aStartLo,1));if(closed){const double wrapLo=std::max(0.0,detail::Below(lengthLo-taskSepHi,1)),wrapHi=std::max(0.0,detail::Above(lengthHi-taskSepLo,1));if(wrapHi<taskSepHi){taskSepLo=wrapLo;taskSepHi=wrapHi;}}
                if(taskSepHi<=4*radiusEnvelopeMM)continue;const double distance=detail::BoxDistance(task.ab,task.bb);if(distance>2*radiusEnvelopeMM+16*PositionalEpsilonMM){anyNonlocal=true;nonlocalLower=std::min(nonlocalLower,distance);continue;}
                // A lower box-distance bound alone cannot prove contact; only
                // an upper witness between two actual points may do so.
                if(taskSepLo>=4*radiusEnvelopeMM&&detail::ContactWitness(task.a,task.b)<=2*radiusEnvelopeMM){result.status=Status::NonlocalContact;return result;}if(task.depth>=MaximumBisectionDepth){result.status=Status::NonlocalClearanceUnproved;return result;}if(pending.size()+2>PendingLimit){result.status=Status::NonlocalClearanceUnproved;return result;}
                if(task.ab.lengthUpper>=task.bb.lengthUpper){auto halves=detail::Split(task.a);auto leftBounds=detail::Bound(halves.first,whole),rightBounds=detail::Bound(halves.second,whole);const double midLo=detail::AddDown(task.aStartLo,leftBounds.lengthLower),midHi=detail::AddUp(task.aStartHi,leftBounds.lengthUpper);pending.push_back({std::move(halves.second),task.b,rightBounds,task.bb,midLo,midHi,task.bStartLo,task.bStartHi,task.depth+1});pending.push_back({std::move(halves.first),std::move(task.b),leftBounds,task.bb,task.aStartLo,task.aStartHi,task.bStartLo,task.bStartHi,task.depth+1});}
                else{auto halves=detail::Split(task.b);auto leftBounds=detail::Bound(halves.first,whole),rightBounds=detail::Bound(halves.second,whole);const double midLo=detail::AddDown(task.bStartLo,leftBounds.lengthLower),midHi=detail::AddUp(task.bStartHi,leftBounds.lengthUpper);pending.push_back({task.a,std::move(halves.second),task.ab,rightBounds,task.aStartLo,task.aStartHi,midLo,midHi,task.depth+1});pending.push_back({std::move(task.a),std::move(halves.first),task.ab,leftBounds,task.aStartLo,task.aStartHi,task.bStartLo,task.bStartHi,task.depth+1});}
            }
        }
        if(!anyNonlocal){nonlocalLower=4*radiusEnvelopeMM;nonlocalUpper=nonlocalLower;}else nonlocalUpper=nonlocalLower;
        CurveCertificate receipt;receipt.lengthMM={lengthLo,lengthHi};receipt.speedPerNormalizedParameter={speedLo,speedHi};receipt.curvaturePerMM={0,curvature};receipt.maximumAbsCoordinateMM={0,extent};receipt.nonlocalCentrelineDistanceMM={nonlocalLower,nonlocalUpper};receipt.holonomyRadians=holonomyRadians;receipt.everyJoinExactG1G2=true;receipt.localBandInjective=curvature*radiusEnvelopeMM<0.25;receipt.fullPairDomainVisited=true;receipt.endpointsExactlyEqual=seamSame;receipt.seamExactG1G2=seamG2;receipt.work={leaves.size(),maximumDepth,pairs,0,MaximumExactArithmeticBits};receipt.homogeneousBernsteinBounds=true;receipt.provenance=CurveProofProvenance::HomogeneousBernsteinExactV1;
        if(!receipt.homogeneousBernsteinBounds){result.status=Status::RegularityUnproved;return result;}result.status=Status::Produced;result.certificate=std::move(receipt);return result;
    } catch(const detail::Budget&){result.status=Status::ExactArithmeticBudget;return result;} catch(...){return result;}
}
} // namespace core3d::spatial_sweep::proof_producer
