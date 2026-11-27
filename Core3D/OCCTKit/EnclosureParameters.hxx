#pragma once

// Enclosure records preserve legacy unframed values exactly. An explicitly
// stored frame, including identity, uses schema2 and is never erased on read.
#include "EnclosureDefinition.hxx"
#include <gp_Mat.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XYZ.hxx>
#include <cmath>
#include <vector>
namespace core3d::enclosure {
inline constexpr int SchemaVersion=1;
inline constexpr int FramedSchemaVersion=2;
inline constexpr std::size_t ScalarCount=9;
inline constexpr std::size_t FramedScalarCount=17;
inline std::size_t ScalarCountForSchema(int schema) noexcept {
    return schema==SchemaVersion ? ScalarCount
        : schema==FramedSchemaVersion ? FramedScalarCount : 0;
}
struct Parameters {
    EnclosureDefinition definition;
    double metersPerUnit=0;
};
inline bool Encode(const Parameters& p,std::vector<double>& output) {
    output.clear();EnclosureDerivedDimensions inspected;
    if (!std::isfinite(p.metersPerUnit) || p.metersPerUnit<=0
        || !InspectEnclosureDefinition(p.definition,inspected)) return false;
    const auto& d=p.definition.dimensions;
    const int schema=p.definition.constructionFrame ? FramedSchemaVersion : SchemaVersion;
    output={double(schema),double(p.definition.plane),d.width,d.depth,d.height,d.wall,d.floor,d.cornerRadius,p.metersPerUnit};
    if (p.definition.constructionFrame) {
        const auto& frame=p.definition.constructionFrame->values;
        output.insert(output.end(),frame.begin(),frame.end());
    }
    return true;
}
inline bool Decode(int schema,const std::vector<double>& values,Parameters& output) {
    output={};
    const auto count=ScalarCountForSchema(schema);
    if (count==0 || values.size()!=count) return false;
    for (double value:values) if (!std::isfinite(value)) return false;
    if (values[0]!=double(schema) || values[1]<0 || values[1]>2 || values[1]!=std::floor(values[1])) return false;
    Parameters candidate;candidate.definition.plane=int(values[1]);
    candidate.definition.dimensions={values[2],values[3],values[4],values[5],values[6],values[7]};
    candidate.metersPerUnit=values[8];
    if (schema==FramedSchemaVersion) {
        candidate.definition.constructionFrame.emplace();
        std::copy(values.begin()+ScalarCount,values.end(),candidate.definition.constructionFrame->values.begin());
    }
    std::vector<double> encoded;
    if (!Encode(candidate,encoded) || encoded!=values) return false;
    output=std::move(candidate);return true;
}

// Edit case from its sheet (round 10 R10P-S5). The solid is built from its corner: 0..width and 0..depth
// in the plane, 0..height along the extrusion, then through the construction frame, then the part's
// placement. A new size therefore kept the corner and the Case slid (Depth 34 -> 30 moved its middle 2 mm).
// This is the one move, in the part's own coordinates (apply it on the right of the placement), that the
// sheet's edit carries so the Case stays where the person put it:
//   Width and Depth keep the middle of the Case's outline;
//   Height keeps the Case's own bottom (no move);
//   a size whose own axis stands upright in the world on a Case that stands on the floor (lowest point at
//   Z 0 within 0.01 mm) keeps the end that rests on the floor, the same rule and limits as Dimensions
//   (DimensionAnchorEnd in TransformInspectorMeasurementController.cpp);
//   an edit that also changes the plane keeps the placement (no move).
// Frame and unit must be the same in both. A false result means the move is not known: the caller refuses.
inline bool SheetEditShift(const Parameters& was,const Parameters& now,const gp_Trsf& placement,
    gp_Vec& shift) noexcept {
    shift=gp_Vec(0,0,0);
    try {
        const EnclosureDefinition& a=was.definition;
        const EnclosureDefinition& b=now.definition;
        if (a.constructionFrame!=b.constructionFrame || was.metersPerUnit!=now.metersPerUnit
            || !std::isfinite(now.metersPerUnit) || now.metersPerUnit<=0) return false;
        if (a.plane!=b.plane) return true;
        gp_Trsf frame;
        if (b.constructionFrame && !b.constructionFrame->Transform(frame)) return false;
        gp_Trsf world=placement;
        world.Multiply(frame);
        const gp_Mat turn=world.HVectorialPart();
        const double scale=world.ScaleFactor();
        const double corner=world.TranslationPart().Z();
        const double floorTolerance=1.0e-5/now.metersPerUnit;
        if (!std::isfinite(scale) || !std::isfinite(corner) || !std::isfinite(floorTolerance)) return false;
        // Own axes of the solid before the frame (1 X, 2 Y, 3 Z), as BuildEnclosureSolidGeometry lays them out.
        const int widthAxis=b.plane==2 ? 2 : 1;
        const int depthAxis=b.plane==0 ? 2 : 3;
        const int heightAxis=b.plane==0 ? 3 : b.plane==1 ? 2 : 1;
        // The share of a size change that the corner moves by: 0 keeps the low end of the own axis (the
        // corner), 0.5 the middle, 1 the high end.
        const auto share=[&](int axis,double size,double otherwise,double& result) -> bool {
            result=otherwise;
            if (!std::isfinite(size)) return false;
            for (int other=1;other<=3;++other) {
                const double up=turn.Value(3,other);
                if (!std::isfinite(up)) return false;
                if (other==axis ? std::abs(std::abs(up)-1.0)>1.0e-9 : std::abs(up)>1.0e-9) return true;
            }
            const double rise=turn.Value(3,axis)*scale; // world Z gained per unit along the own axis
            const double lowest=rise>0 ? corner : corner+rise*size;
            if (!std::isfinite(lowest)) return false;
            if (std::abs(lowest)<=floorTolerance) result=rise>0 ? 0.0 : 1.0;
            return true;
        };
        double widthShare=0.5,depthShare=0.5,heightShare=0.0;
        if (!share(widthAxis,a.dimensions.width,0.5,widthShare)
            || !share(depthAxis,a.dimensions.depth,0.5,depthShare)
            || !share(heightAxis,a.dimensions.height,0.0,heightShare)) return false;
        gp_XYZ local(0,0,0);
        local.SetCoord(widthAxis,widthShare*(a.dimensions.width-b.dimensions.width));
        local.SetCoord(depthAxis,depthShare*(a.dimensions.depth-b.dimensions.depth));
        local.SetCoord(heightAxis,heightShare*(a.dimensions.height-b.dimensions.height));
        gp_Vec move(local);
        move.Transform(frame); // turn and signed scale only: a vector takes no translation
        if (!std::isfinite(move.X()) || !std::isfinite(move.Y()) || !std::isfinite(move.Z())) return false;
        shift=move;
        return true;
    } catch (...) { shift=gp_Vec(0,0,0); return false; }
}
} // namespace core3d::enclosure
