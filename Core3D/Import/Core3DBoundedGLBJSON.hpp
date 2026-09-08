#pragma once
#import <Foundation/Foundation.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

namespace core3d::gltf {
// Private derivative serialization. Bounds each append before allocation and
// checks cancellation while walking strings/containers. Individual NSNumber
// serialization is delegated to Foundation with at most one scalar at a time.
class BoundedGLBJSON final {
public:
    enum class Status { Ready, Invalid, ResourceLimit, Cancelled };
    explicit BoundedGLBJSON(const std::atomic_bool* cancelled) : myCancelled(cancelled) {}
    Status Encode(id root, std::vector<std::uint8_t>& output) {
        output.clear(); myBytes.clear(); myNodes = 0; myStatus = Status::Ready;
        if (![root isKindOfClass:[NSDictionary class]] || !Value(root,0)) {
            if (myStatus == Status::Ready) myStatus = Status::Invalid;
            myBytes.clear(); return myStatus;
        }
        while (myBytes.size() % 4) if (!Append(" ",1)) return myStatus;
        output.swap(myBytes); return Status::Ready;
    }
private:
    static constexpr std::size_t MaximumBytes = 4U * 1024U * 1024U;
    bool Fail(Status status) { if (myStatus == Status::Ready) myStatus = status; return false; }
    bool Check() {
        return !(myCancelled && myCancelled->load(std::memory_order_acquire))
            || Fail(Status::Cancelled);
    }
    bool Append(const char* bytes, std::size_t count) {
        if (!Check()) return false;
        if (count > MaximumBytes - myBytes.size()) return Fail(Status::ResourceLimit);
        // Reserve the final maximum once, so geometric vector growth cannot
        // allocate above the admitted byte ceiling during an append.
        if (myBytes.capacity() == 0) myBytes.reserve(MaximumBytes);
        myBytes.insert(myBytes.end(),bytes,bytes+count); return true;
    }
    bool String(NSString* string) {
        if (!Check()) return false;
        if ([string lengthOfBytesUsingEncoding:NSUTF8StringEncoding] > 256U*1024U)
            return Fail(Status::ResourceLimit);
        NSData* data = [string dataUsingEncoding:NSUTF8StringEncoding];
        if (!data || !Append("\"",1)) return false;
        const auto* bytes = static_cast<const std::uint8_t*>(data.bytes);
        constexpr char hex[] = "0123456789abcdef";
        for (NSUInteger i=0;i<data.length;++i) {
            const auto c=bytes[i];
            if (c=='"' || c=='\\') { const char escaped[]={'\\',char(c)}; if (!Append(escaped,2)) return false; }
            else if (c < 0x20) { const char escaped[]={'\\','u','0','0',hex[c>>4],hex[c&15]}; if (!Append(escaped,6)) return false; }
            else { const char value=char(c); if (!Append(&value,1)) return false; }
        }
        return Append("\"",1);
    }
    bool Value(id value, std::size_t depth) {
        if (!Check()) return false;
        if (depth > 64 || ++myNodes > 250000) return Fail(Status::ResourceLimit);
        if ([value isKindOfClass:[NSString class]]) return String(value);
        if ([value isKindOfClass:[NSNumber class]]) {
            NSError* error = nil;
            NSData* encoded = [NSJSONSerialization dataWithJSONObject:@[value] options:0 error:&error];
            if (!encoded || error || encoded.length < 3 || encoded.length > 130)
                return Fail(Status::Invalid);
            const auto* bytes=static_cast<const char*>(encoded.bytes);
            if (bytes[0]!='[' || bytes[encoded.length-1]!=']') return Fail(Status::Invalid);
            return Append(bytes+1,encoded.length-2);
        }
        if ([value isKindOfClass:[NSNull class]]) return Append("null",4);
        if ([value isKindOfClass:[NSArray class]]) {
            NSArray* array=value;
            if (array.count > 250000-myNodes) return Fail(Status::ResourceLimit);
            if (!Append("[",1)) return false;
            for (NSUInteger i=0;i<array.count;++i) {
                if (i && !Append(",",1)) return false;
                if (!Value(array[i],depth+1)) return false;
            }
            return Append("]",1);
        }
        if ([value isKindOfClass:[NSDictionary class]]) {
            NSDictionary* dictionary=value;
            if (dictionary.count > (250000-myNodes)/2) return Fail(Status::ResourceLimit);
            for (id key in dictionary) if (![key isKindOfClass:[NSString class]]) return Fail(Status::Invalid);
            NSArray* keys=[dictionary.allKeys sortedArrayUsingSelector:@selector(compare:)];
            if (!Check() || !Append("{",1)) return false;
            for (NSUInteger i=0;i<keys.count;++i) {
                if (i && !Append(",",1)) return false;
                if (++myNodes > 250000) return Fail(Status::ResourceLimit);
                if (!String(keys[i]) || !Append(":",1)
                    || !Value(dictionary[keys[i]],depth+1)) return false;
            }
            return Append("}",1);
        }
        return Fail(Status::Invalid);
    }
    const std::atomic_bool* myCancelled;
    std::vector<std::uint8_t> myBytes;
    std::size_t myNodes=0;
    Status myStatus=Status::Ready;
};
} // namespace core3d::gltf
