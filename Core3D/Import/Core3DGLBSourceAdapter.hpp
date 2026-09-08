#pragma once
#include "Core3DGLBReaderSource.hpp"
#include "Core3DBoundedGLBJSON.hpp"
#include <memory>
#include <string>

namespace core3d::gltf {
struct PreparedGLBReader {
    std::shared_ptr<const GLBReaderSource> bytes;
    PreflightResult preflight;
    // preflight.primitiveSources addresses this reader view. Original records
    // remain in the caller's original preflight for supplied tangent reads.
};
enum class GLBAdapterStatus { Ready, Invalid, Cancelled, ResourceLimit, IOFailure };

inline GLBAdapterStatus PrepareGLBReaderSource(
    int descriptor, std::uint64_t size, const PreflightResult& original,
    const std::atomic_bool* cancelled, PreparedGLBReader& output, std::string& message) noexcept {
    output = {}; message.clear();
    @autoreleasepool { @try { try {
        auto fail = [&](GLBAdapterStatus status,const char* reason) { message=reason; return status; };
        auto stopped = [&]() { return cancelled && cancelled->load(std::memory_order_acquire); };
        if (stopped()) return fail(GLBAdapterStatus::Cancelled,"GLB adaptation was cancelled.");
        auto pinned=std::make_shared<GLBReaderSource>(descriptor,size);
        if (!pinned->IsValid() || !original.IsValid())
            return fail(GLBAdapterStatus::Invalid,"The private reader view needs a valid pinned source.");
        const bool supplied=std::any_of(original.primitiveSources.begin(),original.primitiveSources.end(),
            [](const auto& p) { return p.tangent.IsPresent(); });
        if (!supplied) { output={pinned,original}; return GLBAdapterStatus::Ready; }
        if (original.primitiveSources.empty() || original.primitiveSources.size()>2048
            || !original.hasBinaryChunk || original.jsonChunk.length==0
            || original.jsonChunk.length>GLBReaderSource::MaximumJSONBytes
            || original.binaryChunk.offset>size || original.binaryChunk.length>size-original.binaryChunk.offset)
            return fail(GLBAdapterStatus::Invalid,"The supplied-frame adapter source ranges are invalid.");

        std::uint64_t tailSize=0;
        for (const auto& primitive:original.primitiveSources) {
            const auto& position=primitive.position;
            if (!position.IsPresent() || position.elementBytes!=12 || position.componentType!=5126
                || position.count>1500000 || position.stride<12 || position.stride>252
                || position.count*12>GLBReaderSource::MaximumBytes-tailSize)
                return fail(GLBAdapterStatus::ResourceLimit,"Unique GLB primitive positions exceed the private view budget.");
            tailSize+=position.count*12;
        }
        if (original.binaryChunk.length>GLBReaderSource::MaximumBytes-tailSize)
            return fail(GLBAdapterStatus::ResourceLimit,"The private GLB binary view exceeds32MiB.");
        std::vector<std::uint8_t> json(original.jsonChunk.length);
        bool ioFailure=false;
        if (!pinned->Read(original.jsonChunk.offset,json.data(),json.size(),cancelled,ioFailure))
            return fail(stopped()?GLBAdapterStatus::Cancelled:GLBAdapterStatus::IOFailure,"The pinned GLB JSON could not be read.");
        NSError* error=nil;
        id parsed=[NSJSONSerialization JSONObjectWithData:[NSData dataWithBytes:json.data() length:json.size()]
            options:NSJSONReadingMutableContainers error:&error];
        if (error || ![parsed isKindOfClass:[NSMutableDictionary class]])
            return fail(GLBAdapterStatus::Invalid,"The original GLB JSON changed before adaptation.");
        NSMutableDictionary* root=parsed;
        id a=root[@"accessors"],v=root[@"bufferViews"],m=root[@"meshes"],b=root[@"buffers"];
        if (![a isKindOfClass:[NSMutableArray class]] || ![v isKindOfClass:[NSMutableArray class]]
            || ![m isKindOfClass:[NSMutableArray class]] || ![b isKindOfClass:[NSMutableArray class]])
            return fail(GLBAdapterStatus::Invalid,"The original GLB tables changed before adaptation.");
        NSMutableArray* accessors=a; NSMutableArray* views=v; NSMutableArray* meshes=m; NSMutableArray* buffers=b;
        if (buffers.count!=1 || ![buffers[0] isKindOfClass:[NSMutableDictionary class]]
            || accessors.count>100000-original.primitiveSources.size()
            || views.count>100000-original.primitiveSources.size())
            return fail(GLBAdapterStatus::ResourceLimit,"Private GLB table growth exceeds the definition budget.");
        // Mutable original JSON is bounded by source preflight. Added records
        // are constant-sized (<=2048 accessors/views); no arbitrary accessor
        // extras or names are copied once per primitive.
        std::vector<std::uint64_t> offsets; offsets.reserve(original.primitiveSources.size());
        std::uint64_t used=0;
        for (const auto& primitive:original.primitiveSources) {
            if (stopped()) return fail(GLBAdapterStatus::Cancelled,"GLB adaptation was cancelled.");
            if (primitive.mesh>=meshes.count || ![meshes[primitive.mesh] isKindOfClass:[NSDictionary class]])
                return fail(GLBAdapterStatus::Invalid,"A GLB source mesh changed before adaptation.");
            id values=meshes[primitive.mesh][@"primitives"];
            if (![values isKindOfClass:[NSArray class]] || primitive.primitive>=[values count]
                || ![values[primitive.primitive] isKindOfClass:[NSMutableDictionary class]])
                return fail(GLBAdapterStatus::Invalid,"A GLB source primitive changed before adaptation.");
            NSMutableDictionary* item=values[primitive.primitive]; id attributes=item[@"attributes"];
            if (![attributes isKindOfClass:[NSMutableDictionary class]])
                return fail(GLBAdapterStatus::Invalid,"GLB primitive attributes changed before adaptation.");
            NSMutableDictionary* attrs=attributes;
            for (const auto& binding: {std::make_pair(@"POSITION",&primitive.position),
                    std::make_pair(@"NORMAL",&primitive.normal),std::make_pair(@"TEXCOORD_0",&primitive.uv),
                    std::make_pair(@"TANGENT",&primitive.tangent)}) {
                id index=attrs[binding.first];
                if ((binding.second->IsPresent() && (![index isKindOfClass:[NSNumber class]]
                        || [index unsignedLongLongValue]!=binding.second->definition))
                    || (!binding.second->IsPresent() && index!=nil))
                    return fail(GLBAdapterStatus::Invalid,"GLB primitive bindings changed before adaptation.");
            }
            if (![item[@"indices"] isKindOfClass:[NSNumber class]]
                || [item[@"indices"] unsignedLongLongValue]!=primitive.indices.definition
                || primitive.position.definition>=accessors.count)
                return fail(GLBAdapterStatus::Invalid,"GLB primitive indices changed before adaptation.");
            id source=accessors[primitive.position.definition];
            if (![source isKindOfClass:[NSDictionary class]])
                return fail(GLBAdapterStatus::Invalid,"A GLB POSITION accessor changed before adaptation.");
            const auto offset=original.binaryChunk.length+used;
            offsets.push_back(used); used+=primitive.position.count*12;
            const auto view=views.count, accessor=accessors.count;
            [views addObject:@{@"buffer":@0,@"byteOffset":@(offset),@"byteLength":@(primitive.position.count*12),@"target":@34962}];
            NSMutableDictionary* replacement=[@{@"bufferView":@(view),@"componentType":@5126,
                @"count":@(primitive.position.count),@"type":@"VEC3"} mutableCopy];
            for (NSString* key in @[@"min",@"max"]) if (source[key]) {
                id bounds=source[key];
                if (![bounds isKindOfClass:[NSArray class]] || [bounds count]!=3)
                    return fail(GLBAdapterStatus::Invalid,"GLB POSITION bounds changed before adaptation.");
                replacement[key]=bounds;
            }
            [accessors addObject:replacement]; attrs[@"POSITION"]=@(accessor); [attrs removeObjectForKey:@"TANGENT"];
        }
        if (used!=tailSize) return fail(GLBAdapterStatus::Invalid,"The GLB private layout changed.");
        buffers[0][@"byteLength"]=@(original.binaryChunk.length+tailSize);
        BoundedGLBJSON serializer(cancelled);
        const auto serialized=serializer.Encode(root,json);
        if (serialized!=BoundedGLBJSON::Status::Ready)
            return fail(serialized==BoundedGLBJSON::Status::Cancelled?GLBAdapterStatus::Cancelled:
                serialized==BoundedGLBJSON::Status::ResourceLimit?GLBAdapterStatus::ResourceLimit:GLBAdapterStatus::Invalid,
                "The generated GLB JSON exceeds its bounded representation.");
        if (json.size()+28>GLBReaderSource::MaximumBytes-original.binaryChunk.length-tailSize)
            return fail(GLBAdapterStatus::ResourceLimit,"The generated GLB reader view exceeds32MiB.");
        const auto binaryOffset=json.size()+28;
        const auto total=binaryOffset+original.binaryChunk.length+tailSize;
        std::vector<std::uint8_t> prefix(binaryOffset,0);
        auto write32=[](std::uint8_t* p,std::uint32_t n) { for (unsigned i=0;i<4;++i)p[i]=std::uint8_t(n>>(8*i)); };
        write32(prefix.data(),0x46546c67);write32(prefix.data()+4,2);write32(prefix.data()+8,std::uint32_t(total));
        write32(prefix.data()+12,std::uint32_t(json.size()));write32(prefix.data()+16,0x4e4f534a);
        std::memcpy(prefix.data()+20,json.data(),json.size());
        write32(prefix.data()+20+json.size(),std::uint32_t(original.binaryChunk.length+tailSize));
        write32(prefix.data()+24+json.size(),0x004e4942);
        std::vector<std::uint8_t> tail(tailSize);
        for (std::size_t k=0;k<original.primitiveSources.size();++k) {
            const auto& source=original.primitiveSources[k].position;
            for (std::uint64_t first=0;first<source.count;) {
                const auto count=std::min<std::uint64_t>(4096,source.count-first);
                std::vector<std::uint8_t> block((count-1)*source.stride+12);
                if (!pinned->Read(source.offset+first*source.stride,block.data(),block.size(),cancelled,ioFailure))
                    return fail(stopped()?GLBAdapterStatus::Cancelled:GLBAdapterStatus::IOFailure,"Pinned GLB positions could not be copied exactly.");
                for (std::uint64_t n=0;n<count;++n)
                    std::memcpy(tail.data()+offsets[k]+(first+n)*12,block.data()+n*source.stride,12);
                first+=count;
            }
        }
        auto adapted=std::make_shared<GLBReaderSource>(descriptor,size,original.binaryChunk,std::move(prefix),std::move(tail));
        if (!adapted->IsValid()) return fail(GLBAdapterStatus::Invalid,"The private GLB view failed its final range validation.");
        PreflightResult view=original;view.jsonChunk={20,json.size()};
        view.binaryChunk={binaryOffset,original.binaryChunk.length+tailSize};
        for (auto& image:view.embeddedImages)
            if (!adapted->MapOriginalBinaryRange(image.bytes,image.bytes))
                return fail(GLBAdapterStatus::Invalid,"An embedded image lost its original GLB byte range.");
        for (std::size_t k=0;k<view.primitiveSources.size();++k) {
            auto& primitive=view.primitiveSources[k];
            for (auto* record:{&primitive.position,&primitive.normal,&primitive.uv,&primitive.indices,&primitive.tangent}) {
                if (!record->IsPresent()) continue;
                ByteRange first;
                if (!adapted->MapOriginalBinaryRange({record->offset,record->elementBytes},first)
                    || !adapted->MapOriginalBinaryRange(record->view,record->view))
                    return fail(GLBAdapterStatus::Invalid,"An accessor lost its original GLB byte range.");
                record->offset=first.offset;
            }
            primitive.position.view={adapted->TailOffset()+offsets[k],primitive.position.count*12};
            primitive.position.offset=primitive.position.view.offset;primitive.position.stride=12;
            // TANGENT is deliberately absent from the derivative parser view;
            // its original record remains authoritative in the caller.
            primitive.tangent={};
        }
        output={adapted,std::move(view)};return GLBAdapterStatus::Ready;
    } catch (const std::bad_alloc&) { message="The GLB private view exceeded available memory.";return GLBAdapterStatus::ResourceLimit;
    } catch (...) { message="The GLB private view failed safely.";return GLBAdapterStatus::Invalid;
    } } @catch (NSException* exception) { (void)exception; message="GLB adaptation rejected an exceptional JSON value.";return GLBAdapterStatus::Invalid; } }
}
} // namespace core3d::gltf
