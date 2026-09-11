#pragma once

// Detached OCCT geometry only. Document admission,
// persistence, single commit and recovery must be supplied by the native worker.
#include "EnclosureDefinition.hxx"
#include "ProfileCurveFace.hxx"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Bnd_Box.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Standard_ErrorHandler.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_ListOfShape.hxx>
#include <memory>

namespace core3d {
class EnclosureCancellationIndicator final : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(EnclosureCancellationIndicator,Message_ProgressIndicator)
public:
    explicit EnclosureCancellationIndicator(std::shared_ptr<std::atomic_bool> value)
        : cancellation(std::move(value)) {}
protected:
    Standard_Boolean UserBreak() override { return cancellation && cancellation->load(); }
    void Show(const Message_ProgressScope&,const Standard_Boolean) override {}
private:
    std::shared_ptr<std::atomic_bool> cancellation;
};
struct EnclosureSolidResult {
    TopoDS_Shape solid;
    std::array<double,6> bounds{};
    double volume=0;
};
inline bool BuildEnclosureSolidGeometry(const EnclosureDefinition& definition,
    const std::shared_ptr<std::atomic_bool>& cancelled, EnclosureSolidResult& output) noexcept {
    output={};
    try {
        OCC_CATCH_SIGNALS
        if (!cancelled || cancelled->load()) return false;
        EnclosureProfileDependency dependency;
        if (!DeriveEnclosureProfiles(definition,dependency)) return false;
        const auto direction=[&](double length) {
            return definition.plane==0 ? gp_Vec(0,0,length)
                : definition.plane==1 ? gp_Vec(0,length,0) : gp_Vec(length,0,0);
        };
        const auto extrude=[&](const ProfileDefinition& profile, double offset, TopoDS_Shape& result) {
            if (cancelled->load() || !profile.curves) return false;
            ProfileCurveFaceResult face;
            if (!BuildProfileCurveFace(*profile.curves,profile.plane,*cancelled,face)) return false;
            BRepPrimAPI_MakePrism prism(face.face,direction(profile.depth),Standard_True,Standard_True);
            if (cancelled->load() || !prism.IsDone()) return false;
            result=prism.Shape();
            if (offset!=0) {
                gp_Trsf placement;placement.SetTranslation(direction(offset));
                BRepBuilderAPI_Transform moved(result,placement,Standard_True,Standard_False);
                if (cancelled->load() || !moved.IsDone()) return false;
                result=moved.Shape();
            }
            if (result.IsNull() || result.ShapeType()!=TopAbs_SOLID) return false;
            auto solid=TopoDS::Solid(result);
            if (!BRepLib::OrientClosedSolid(solid)
                || !BRepCheck_Analyzer(solid,Standard_True).IsValid() || cancelled->load()) return false;
            result=solid;return true;
        };
        TopoDS_Shape outer,cavity;
        if (!extrude(dependency.outer,0,outer)
            || !extrude(dependency.cavity,dependency.cavityOffset,cavity)) return false;
        TopTools_ListOfShape arguments,tools;arguments.Append(outer);tools.Append(cavity);
        BRepAlgoAPI_Cut cut;cut.SetRunParallel(Standard_False);cut.SetNonDestructive(Standard_True);
        cut.SetArguments(arguments);cut.SetTools(tools);cut.SetCheckInverted(Standard_True);
        Handle(EnclosureCancellationIndicator) progress=new EnclosureCancellationIndicator(cancelled);
        cut.Build(progress->Start());
        if (cancelled->load() || !cut.IsDone() || cut.Shape().IsNull()) return false;
        TopoDS_Shape shape=cut.Shape();
        // OCCT may wrap its one result in a compound. No extra free topology is
        // discarded: only a compound containing exactly one solid is admitted.
        if (shape.ShapeType()==TopAbs_COMPOUND) {
            TopoDS_Iterator child(shape);if (!child.More()) return false;
            const auto single=child.Value();child.Next();if (child.More()) return false;
            shape=single;
        }
        if (shape.ShapeType()!=TopAbs_SOLID) return false;
        auto solid=TopoDS::Solid(shape);
        if (!BRepLib::OrientClosedSolid(solid) || !BRepCheck_Analyzer(solid,Standard_True).IsValid()) return false;
        GProp_GProps properties;BRepGProp::VolumeProperties(solid,properties);
        const double volume=properties.Mass(),expected=dependency.expectedVolume;
        if (!std::isfinite(volume) || volume<=0 || std::abs(volume-expected)>std::max(1e-8,expected*1e-8)) return false;
        Bnd_Box box;BRepBndLib::AddOptimal(solid,box,Standard_False,Standard_False);
        if (box.IsVoid() || box.IsOpen()) return false;
        EnclosureSolidResult result;box.Get(result.bounds[0],result.bounds[1],result.bounds[2],result.bounds[3],result.bounds[4],result.bounds[5]);
        const auto& d=definition.dimensions;
        const std::array<double,3> dimensions=definition.plane==0 ? std::array<double,3>{d.width,d.depth,d.height}
            : definition.plane==1 ? std::array<double,3>{d.width,d.height,d.depth} : std::array<double,3>{d.height,d.width,d.depth};
        for (std::size_t axis=0;axis<3;++axis)
            if (!std::isfinite(result.bounds[axis]) || !std::isfinite(result.bounds[axis+3])
                || std::abs(result.bounds[axis])>1e-7 || std::abs(result.bounds[axis+3]-dimensions[axis])>1e-7) return false;
        double finalVolume=volume;
        if (definition.constructionFrame) {
            gp_Trsf frame;
            if (cancelled->load() || !definition.constructionFrame->Transform(frame)) return false;
            BRepBuilderAPI_Transform transformed(solid,frame,Standard_True,Standard_False);
            if (cancelled->load() || !transformed.IsDone() || transformed.Shape().IsNull()
                || transformed.Shape().ShapeType()!=TopAbs_SOLID) return false;
            solid=TopoDS::Solid(transformed.Shape());
            if (!BRepLib::OrientClosedSolid(solid)
                || !BRepCheck_Analyzer(solid,Standard_True).IsValid()) return false;
            GProp_GProps framedProperties;BRepGProp::VolumeProperties(solid,framedProperties);
            finalVolume=framedProperties.Mass();
            const double framedExpected=expected*definition.constructionFrame->AbsoluteVolumeScale();
            if (!std::isfinite(finalVolume) || finalVolume<=0 || !std::isfinite(framedExpected)
                || framedExpected<=0 || std::abs(finalVolume-framedExpected)>std::max(1e-8,framedExpected*1e-8)) return false;
            Bnd_Box framedBox;BRepBndLib::AddOptimal(solid,framedBox,Standard_False,Standard_False);
            if (framedBox.IsVoid() || framedBox.IsOpen()) return false;
            framedBox.Get(result.bounds[0],result.bounds[1],result.bounds[2],result.bounds[3],result.bounds[4],result.bounds[5]);
            // The outer rounded footprint is a rectangle plus a radius disk.
            // Its support function gives tight transformed extrema, including
            // rotations for which transformed enclosing-box corners are absent.
            const int u=definition.plane==2 ? 2 : 1;
            const int v=definition.plane==0 ? 2 : 3;
            const int w=definition.plane==0 ? 3 : definition.plane==1 ? 2 : 1;
            const double tolerance=1e-7*std::max(1.0,std::abs(frame.ScaleFactor()));
            for (int axis=0;axis<3;++axis) {
                const double a=frame.Value(axis+1,u),b=frame.Value(axis+1,v),c=frame.Value(axis+1,w);
                const double center=frame.Value(axis+1,4)+a*d.width/2+b*d.depth/2+c*d.height/2;
                const double extent=std::abs(a)*(d.width/2-d.cornerRadius)
                    +std::abs(b)*(d.depth/2-d.cornerRadius)+std::hypot(a,b)*d.cornerRadius
                    +std::abs(c)*d.height/2;
                if (!std::isfinite(center) || !std::isfinite(extent)
                    || !std::isfinite(result.bounds[axis]) || !std::isfinite(result.bounds[axis+3])
                    || std::abs(result.bounds[axis]-(center-extent))>tolerance
                    || std::abs(result.bounds[axis+3]-(center+extent))>tolerance) return false;
            }
        }
        if (cancelled->load()) return false;
        result.solid=solid;result.volume=finalVolume;output=std::move(result);return true;
    } catch (...) {output={};return false;}
}
} // namespace core3d
