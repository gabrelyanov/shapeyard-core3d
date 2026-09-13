#pragma once
#if DEBUG
#include "RetainedSolidBinaryDriver.hxx"
#include "ReceiptCatalogProbe.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <PCDM_StoreStatus.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <sstream>
#include <stdexcept>
#include <map>

namespace core3d::retained_solid {
struct Probe {
    struct App {
        Handle(TDocStd_Application) app=new TDocStd_Application();
        Handle(TDocStd_Document) doc;
        ~App(){try{if(!doc.IsNull())app->Close(doc);}catch(...) {}}
    };
    struct Fixture {TDF_Label owner,record;Envelope envelope;TopoDS_Shape base;std::vector<std::uint8_t> bytes;};
    static void Identity(const TDF_Label& label,const Standard_GUID& id,const UUID& value){
        TDataStd_AsciiString::Set(label,id,UUIDText(value).c_str());
    }
    static Handle(Attribute) Install(const Handle(TDocStd_Document)& doc,const TDF_Label& label,
                                      const Envelope& envelope,const TopoDS_Shape& base){
        if(doc.IsNull()||!doc->HasOpenCommand()||label.Data()!=doc->GetData())throw std::invalid_argument("probe command");
        std::vector<std::uint8_t> bytes;if(!Encode(envelope,bytes))throw std::invalid_argument("probe envelope");
        Handle(Attribute) attribute;
        if(!label.FindAttribute(AttributeID(),attribute)){attribute=new Attribute();label.AddAttribute(attribute);}
        auto value=std::make_shared<Payload>();value->envelope=envelope;value->bytes=std::move(bytes);value->base=base;
        attribute->Backup();attribute->value_=std::move(value);return attribute;
    }
    static Fixture New(App& holder,bool xcaf,int version,double unit=.001){
        Core3DDefineSafeBinXCAFFormat(holder.app);holder.app->NewDocument(xcaf?"BinXCAF":"BinOcaf",holder.doc);
        if(holder.doc.IsNull())throw std::invalid_argument("probe document");
        auto doc=holder.doc;doc->ChangeStorageFormatVersion(TDocStd_FormatVersion(version));
        auto tool=XCAFDoc_DocumentTool::ShapeTool(doc->Main());(void)XCAFDoc_DocumentTool::ColorTool(doc->Main());
        (void)XCAFDoc_DocumentTool::VisMaterialTool(doc->Main());XCAFDoc_DocumentTool::SetLengthUnit(doc,unit);
        Fixture f;auto& e=f.envelope;e.document.fill(1);e.entity.fill(2);e.definition.fill(3);e.sourceFeature.fill(4);e.derivedFeature.fill(5);
        Identity(doc->Main(),DocumentIdentifierAttributeID(),e.document);
        const double n=.001/unit;auto current=BRepPrimAPI_MakeBox(80*n,60*n,6*n).Shape();
        BRepBuilderAPI_Copy copy(current,Standard_True,Standard_False);f.base=copy.Shape();
        f.owner=tool->AddShape(current,Standard_False);if(f.owner.IsNull())throw std::invalid_argument("probe owner");
        Identity(f.owner,EntityIdentifierAttributeID(),e.entity);Identity(f.owner,DefinitionIdentifierAttributeID(),e.definition);
        TDataStd_Integer::Set(f.owner,GeometryRepresentationAttributeID(),1);
        profile::Parameters source;source.metersPerUnit=unit;source.definition.depth=6*n;
        source.definition.points={{-0.0,0},{80*n,0},{80*n,60*n},{0,60*n}};
        if(!profile::Encode(source,e.sourceValues))throw std::invalid_argument("probe source recipe");
        e.sourceSchema=profile::SchemaFor(source);e.sourceFamily=1;e.metersPerUnit=unit;e.radius=3*n;e.point={10*n,30*n,-0.0};
        if(!Encode(e,f.bytes))throw std::invalid_argument("probe encode");
        doc->SetUndoLimit(40);doc->ClearUndos();doc->NewCommand();f.record=f.owner.FindChild(13,true);
        TNaming_Builder(f.record).Select(current,current);Install(doc,f.record,e,f.base);
        if(!Core3DValidateRetainedSolidDocument(doc)||!doc->CommitCommand())throw std::invalid_argument("probe fixture admission");
        return f;
    }
    static std::string Save(App& holder){
        std::ostringstream out(std::ios::out|std::ios::binary);
        if(holder.app->SaveAs(holder.doc,out)!=PCDM_SS_OK)throw std::invalid_argument("probe save");
        return out.str();
    }
    static bool Open(const std::string& bytes,bool oldReader,std::vector<Record>* result=nullptr,
                     std::size_t envelopeLimit=MaximumAggregateEnvelopeBytes,bool requireV3=false,int roleFault=0){
        App holder;Core3DDefineSafeBinXCAFFormat(holder.app);
        if(oldReader||envelopeLimit!=MaximumAggregateEnvelopeBytes||roleFault){
            for(bool xcaf:{false,true}){
                auto* reader=new Core3DBoundedBinXCAFRetrievalDriver();if(oldReader)reader->DebugRejectRetainedSolidType();
                reader->DebugSetRetainedEnvelopeLimit(envelopeLimit);reader->DebugSetRetainedRoleFault(roleFault);
                Handle(PCDM_StorageDriver) writer=xcaf?Handle(PCDM_StorageDriver)(new BinXCAFDrivers_DocumentStorageDriver())
                    :Handle(PCDM_StorageDriver)(new BinDrivers_DocumentStorageDriver());
                holder.app->DefineFormat(xcaf?"BinXCAF":"BinOcaf","Historical type table",xcaf?"xbf":"cbf",reader,writer);
            }
        }
        try{
            std::istringstream in(bytes,std::ios::in|std::ios::binary);
            Core3DBeginSafeBinaryRead();const auto status=holder.app->Open(in,holder.doc);
            if(status!=PCDM_RS_OK||Core3DSafeBinaryReadWasRejected()||holder.doc.IsNull())return false;
            std::vector<Record> records;if(!ReadAll(holder.doc,records)||records.empty()
                ||!Core3DValidateRetainedSolidDocument(holder.doc))return false;
            if(requireV3){core3d::receipt::Catalog catalog;
                if(core3d::receipt::Read(holder.doc,catalog)!=core3d::receipt::ReadStatus::Valid||!catalog.tree||catalog.count()!=1)return false;
            }
            if(result)*result=std::move(records);return true;
        }catch(...){return false;}
    }
    static bool BoxGeometry(const TopoDS_Shape& shape,double unit,double scale=1){
        Bnd_Box box;BRepBndLib::AddOptimal(shape,box,Standard_False,Standard_False);
        if(box.IsVoid()||box.IsWhole()||box.IsOpen())return false;
        std::array<double,6> b{};box.Get(b[0],b[1],b[2],b[3],b[4],b[5]);
        const double mm=unit*1000;const std::array<double,6> expected{0,0,0,80,60,6};
        for(std::size_t i=0;i<6;++i)if(!std::isfinite(b[i])||std::abs(b[i]*mm-expected[i]*scale)>1e-8)return false;
        TopTools_MapOfShape vertices;for(TopExp_Explorer it(shape,TopAbs_VERTEX);it.More();it.Next())vertices.Add(it.Current());
        return vertices.Extent()==8&&std::abs(Volume(shape)*mm*mm*mm-28800*scale*scale*scale)<1e-7;
    }
    static double Volume(const TopoDS_Shape& shape){GProp_GProps p;BRepGProp::VolumeProperties(shape,p);return p.Mass();}
    static std::string Raw(const TopoDS_Shape& shape){std::ostringstream out;BRepTools::Write(shape,out,false,false,TopTools_FormatVersion_VERSION_3);return out.str();}
    static void Rehash(std::string& file,std::size_t offset,std::size_t count){
        std::vector<std::uint8_t> body(file.begin()+offset,file.begin()+offset+count-32);Digest hash;
        if(!Hash(body,hash))throw std::invalid_argument("probe hash");
        std::copy(hash.begin(),hash.end(),file.begin()+offset+count-32);
    }
    static std::map<std::string,bool> Run(int scenario){
        std::map<std::string,bool> checks;
        try{
            if(scenario==0||scenario==1){
                for(bool xcaf:{false,true})for(int version:{10,11,12}){
                    const std::string prefix=std::string(xcaf?"xcaf":"ocaf")+"."+std::to_string(version)+".";
                    App holder;const auto f=New(holder,xcaf,version);const auto bytes=Save(holder);
                    std::vector<Record> records;const bool opened=Open(bytes,false,&records);
                    checks[prefix+"roundtrip"]=opened&&records.size()==1&&records[0].value->bytes==f.bytes
                        &&BoxGeometry(records[0].value->base,.001);
                    if(scenario==0){
                        checks[prefix+"oldReaderRefuses"]=!Open(bytes,true);
                        {
                            using RP=core3d::receipt::v3::DebugProbe;
                            auto tree=RP::EmptyFor(holder.doc);tree=tree->append(RP::Fixture(1,tree->document()));
                            holder.doc->NewCommand();RP::Install(holder.doc,tree);
                            if(!holder.doc->CommitCommand())throw std::invalid_argument("probe mixed receipt commit");
                            if(version<12){
                                bool refused=false;try{(void)Save(holder);}catch(...){refused=true;}
                                checks[prefix+"mixedLegacyVersionRefuses"]=refused;
                                continue;
                            }
                            const auto mixed=Save(holder);std::vector<Record> mixedRecords;
                            checks[prefix+"mixedV3"]=Open(mixed,false,&mixedRecords,MaximumAggregateEnvelopeBytes,true)&&mixedRecords.size()==1
                                &&mixedRecords[0].value->bytes==f.bytes;
                            checks[prefix+"mixedOldReaderRefuses"]=!Open(mixed,true);
                            for(int fault=1;fault<=3;++fault)checks[prefix+"roleFault"+std::to_string(fault)]=
                                !Open(mixed,false,nullptr,MaximumAggregateEnvelopeBytes,true,fault);
                        }
                        continue;
                    }
                    const std::string wire(reinterpret_cast<const char*>(f.bytes.data()),f.bytes.size());
                    const auto offset=bytes.find(wire);if(offset==std::string::npos||bytes.find(wire,offset+1)!=std::string::npos)
                        throw std::invalid_argument("probe unique envelope");
                    for(int mutation=0;mutation<10;++mutation){
                        auto bad=bytes;
                        if(mutation==0)bad[offset+4]=2; // future envelope version
                        if(mutation==1)bad[offset+8]=3; // unsupported source family
                        if(mutation==2)bad[offset+9]=3; // non XYZ axis
                        if(mutation==3)std::copy_n(bad.begin()+offset+58,16,bad.begin()+offset+74); // duplicate feature UUIDs
                        if(mutation==4)std::fill_n(bad.begin()+offset+10,16,char(0)); // missing document UUID
                        if(mutation==5)std::fill_n(bad.begin()+offset+146,8,char(0x7f)); // forged count before allocation
                        if(mutation==6)bad[offset+f.bytes.size()-1]^=1; // actual digest mismatch
                        if(mutation==7)bad.resize(offset+f.bytes.size()-1); // truncated actual file
                        if(mutation==8){
                            const std::string type="Core3D_RetainedSolid";const auto at=bad.find(type);
                            if(at==std::string::npos)throw std::invalid_argument("probe stored type");
                            bad[at]='X'; // unknown runtime source type, same byte length
                        }
                        if(mutation==9){std::fill_n(bad.begin()+offset+138,8,char(0));Rehash(bad,offset,f.bytes.size());}
                        if(mutation<6)Rehash(bad,offset,f.bytes.size());
                        checks[prefix+"mutation"+std::to_string(mutation)]=!Open(bad,false);
                    }
                }
            }else if(scenario==2){
                for(double unit:{.001,1.0}){
                    const auto prefix=unit==.001?"mm.":"metre.";App holder;const auto f=New(holder,true,12,unit);
                    auto doc=holder.doc;Handle(Attribute) attribute;f.record.FindAttribute(AttributeID(),attribute);
                    const auto original=attribute->value();const auto raw=Raw(original->base);
                    auto changed=f.envelope;changed.radius*=1.5;doc->NewCommand();Install(doc,f.record,changed,f.base);
                    if(!doc->CommitCommand())throw std::invalid_argument("probe change");
                    checks[std::string(prefix)+"undo"]=doc->Undo()&&attribute->value()==original&&attribute->value()->bytes==f.bytes;
                    checks[std::string(prefix)+"redo"]=doc->Redo()&&Bits(attribute->value()->envelope.radius)==Bits(changed.radius)
                        &&attribute->value()->base.IsEqual(f.base);
                    const auto beforeAbort=attribute->value();doc->NewCommand();Install(doc,f.record,f.envelope,f.base);doc->AbortCommand();
                    checks[std::string(prefix)+"abort"]=attribute->value()==beforeAbort;
                    gp_Trsf rotation;rotation.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),.37);
                    gp_Trsf move;move.SetTranslation(gp_Vec(17,23,31));
                    const gp_Trsf transform=move*rotation;
                    const auto currentBefore=XCAFDoc_ShapeTool::GetShape(f.owner);
                    const TopLoc_Location displacement(transform);
                    const auto expectedCurrent=currentBefore.Moved(displacement);
                    doc->NewCommand();TNaming::Displace(f.owner,displacement);
                    const auto currentAfter=XCAFDoc_ShapeTool::GetShape(f.owner);
                    checks[std::string(prefix)+"transformBaseExact"]=attribute->value()==beforeAbort
                        &&Raw(attribute->value()->base)==raw
                        &&!currentAfter.IsEqual(currentBefore)&&currentAfter.IsEqual(expectedCurrent)
                        &&Core3DValidateRetainedSolidDocument(doc);
                    doc->AbortCommand();checks[std::string(prefix)+"transformAbort"]=Core3DValidateRetainedSolidDocument(doc)
                        &&Raw(attribute->value()->base)==raw
                        &&XCAFDoc_ShapeTool::GetShape(f.owner).IsEqual(currentBefore);
                    // OCCT Displace intentionally rejects scale. Its geometry
                    // Transform walker also visits NamedShape only, and must
                    // not scale the historical custom attribute's solid.
                    gp_Trsf scale;scale.SetScale(gp_Pnt(0,0,0),2);
                    doc->NewCommand();TNaming::Transform(f.owner,scale);
                    checks[std::string(prefix)+"scaleBaseExact"]=attribute->value()==beforeAbort
                        &&Raw(attribute->value()->base)==raw
                        &&BoxGeometry(XCAFDoc_ShapeTool::GetShape(f.owner),unit,2)
                        &&Core3DValidateRetainedSolidDocument(doc);
                    doc->AbortCommand();checks[std::string(prefix)+"scaleAbort"]=Core3DValidateRetainedSolidDocument(doc)
                        &&Raw(attribute->value()->base)==raw
                        &&XCAFDoc_ShapeTool::GetShape(f.owner).IsEqual(currentBefore);
                    const auto stored=Save(holder);std::vector<Record> reopened;
                    checks[std::string(prefix)+"fresh"]=Open(stored,false,&reopened)&&reopened.size()==1
                        &&reopened[0].value->bytes==beforeAbort->bytes
                        &&BoxGeometry(reopened[0].value->base,unit);
                }
            }else if(scenario==3){
                App holder;auto f=New(holder,true,12);auto doc=holder.doc;
                Handle(Attribute) attribute;f.record.FindAttribute(AttributeID(),attribute);const auto original=attribute->value();
                auto empty=Handle(Attribute)::DownCast(attribute->NewEmpty());
                checks["newEmpty"]=!empty.IsNull()&&!empty->value();empty->Restore(attribute);
                checks["restoreSharesImmutable"]=empty->value()==original;
                App foreign;auto other=New(foreign,true,12);Handle(Attribute) destination;other.record.FindAttribute(AttributeID(),destination);
                foreign.doc->NewCommand();attribute->Paste(destination,new TDF_RelocationTable());
                // Change actual owner identity so copied exact IDs cannot grant ownership.
                UUID foreignEntity;foreignEntity.fill(29);Identity(other.owner,EntityIdentifierAttributeID(),foreignEntity);
                checks["foreignOwnerRefused"]=!Core3DValidateRetainedSolidDocument(foreign.doc);
                foreign.doc->AbortCommand();checks["foreignAbortRestores"]=Core3DValidateRetainedSolidDocument(foreign.doc);
                const auto singleFile=Save(holder);
                checks["singleEnvelopeBudgetPositive"]=Open(singleFile,false,nullptr,f.bytes.size());
                auto sharedOriginal=original->base;auto tool=XCAFDoc_DocumentTool::ShapeTool(doc->Main());doc->NewCommand();
                gp_Trsf t;t.SetTranslation(gp_Vec(200,0,0));auto current=sharedOriginal.Moved(TopLoc_Location(t));
                const auto second=tool->AddShape(current,Standard_False);auto e=f.envelope;e.entity.fill(30);e.definition.fill(31);e.sourceFeature.fill(32);e.derivedFeature.fill(33);
                Identity(second,EntityIdentifierAttributeID(),e.entity);Identity(second,DefinitionIdentifierAttributeID(),e.definition);
                TDataStd_Integer::Set(second,GeometryRepresentationAttributeID(),1);const auto record=second.FindChild(13,true);
                TNaming_Builder(record).Select(current,current);Install(doc,record,e,sharedOriginal);
                checks["sharedReferencesAdmitted"]=Core3DValidateRetainedSolidDocument(doc);
                if(!doc->CommitCommand())throw std::invalid_argument("probe shared commit");
                const auto sharedFile=Save(holder);
                std::vector<Record> reopened;checks["sharedCodecIdentity"]=Open(sharedFile,false,&reopened)&&reopened.size()==2
                    &&reopened[0].value->base.IsEqual(reopened[1].value->base);
                const std::size_t jointBytes=f.bytes.size()*2;
                checks["aggregateExactBudgetPositive"]=Open(sharedFile,false,nullptr,jointBytes);
                checks["aggregateBudgetRefused"]=!Open(sharedFile,false,nullptr,jointBytes-1);
                doc->NewCommand();e.sourceFeature=f.envelope.sourceFeature;Install(doc,record,e,sharedOriginal);
                checks["duplicateNamespaceRefused"]=!Core3DValidateRetainedSolidDocument(doc);doc->AbortCommand();
                checks["namespaceAbortRestores"]=Core3DValidateRetainedSolidDocument(doc);
                doc->NewCommand();Install(doc,f.record,f.envelope,f.base.Reversed());
                checks["reversedBaseRefused"]=!Core3DValidateRetainedSolidDocument(doc);
                bool saveRefused=false;try{(void)Save(holder);}catch(...){saveRefused=true;}
                checks["invalidBaseWriteRefused"]=saveRefused;doc->AbortCommand();
                checks["invalidBaseAbortRestores"]=Core3DValidateRetainedSolidDocument(doc)&&attribute->value()==original;
            }else checks["invalidScenario"]=false;
        }catch(const Standard_Failure&){checks["nativeSetupFailure"]=false;}
         catch(...){checks["setupFailure"]=false;}
        return checks;
    }
};
} // namespace core3d::retained_solid
#endif
