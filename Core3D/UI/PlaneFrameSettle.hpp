// Shared by Push-pull (ShapeInteractor.cpp) and flat Bevel (BevelOperationController.cpp):
// settle slanted plane frames so a part saves and reopens to the same bytes.
#pragma once

#include <BRep_Tool.hxx>
#include <Geom_Plane.hxx>
#include <Standard_Real.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace core3d {
	namespace plane_frames {
	// What the binary reader makes of a stored plane frame: each direction is
	// re-normalised and the X direction is rebuilt as N x (X x N) (OCCT 7.8
	// BinTools_SurfaceSet). A frame is file-stable when that gives it back bit
	// for bit.
	inline Standard_Boolean SameBits(const Standard_Real a, const Standard_Real b) noexcept {
		return std::memcmp(&a, &b, sizeof(Standard_Real)) == 0;
	}
	inline Standard_Boolean SameDirBits(const gp_Dir& a, const gp_Dir& b) noexcept {
		return SameBits(a.X(), b.X()) && SameBits(a.Y(), b.Y())
			&& SameBits(a.Z(), b.Z());
	}
	inline gp_Ax3 ReadBackPlaneFrame(const gp_Ax3& frame) {
		const gp_Dir n(frame.Direction().X(), frame.Direction().Y(), frame.Direction().Z());
		const gp_Dir vx(frame.XDirection().X(), frame.XDirection().Y(), frame.XDirection().Z());
		const gp_Dir vy(frame.YDirection().X(), frame.YDirection().Y(), frame.YDirection().Z());
		gp_Ax3 back(frame.Location(), n, vx);
		if (vy.DotCross(n, vx) < 0) back.YReverse();
		return back;
	}
	inline Standard_Boolean PlaneFrameIsFileStable(const gp_Ax3& frame) {
		const gp_Ax3 back = ReadBackPlaneFrame(frame);
		return SameDirBits(back.Direction(), frame.Direction())
			&& SameDirBits(back.XDirection(), frame.XDirection())
			&& SameDirBits(back.YDirection(), frame.YDirection());
	}

	// A slanted plane (flat Bevel) can flip between two frames on every save
	// and never settle. Move such a frame's X direction by at most two units in
	// the last place per component (about 1e-16 of a turn: nothing a pcurve or
	// a tolerance can see) to the nearest frame the reader gives back unchanged.
	// Returns the number of planes that could not be settled.
	inline Standard_Integer SettlePlaneFramesForFile(const TopoDS_Shape& shape) {
		Standard_Integer unsettled = 0;
		TopTools_IndexedMapOfShape faces;
		TopExp::MapShapes(shape, TopAbs_FACE, faces);
		for (Standard_Integer index = 1; index <= faces.Extent(); ++index) {
			TopLoc_Location location;
			const Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(
				BRep_Tool::Surface(TopoDS::Face(faces(index)), location));
			if (plane.IsNull()) continue;
			const gp_Ax3 frame = plane->Position();
			if (PlaneFrameIsFileStable(frame)) continue;
			const gp_Ax3 read = ReadBackPlaneFrame(frame);
			if (PlaneFrameIsFileStable(read)) {
				plane->SetPosition(read);
				continue;
			}
			const gp_Dir n = read.Direction();
			const Standard_Real base[3] = {
				read.XDirection().X(), read.XDirection().Y(), read.XDirection().Z()};
			Standard_Boolean found = Standard_False;
			for (Standard_Integer reach = 1; reach <= 2 && !found; ++reach) {
				for (Standard_Integer step = 0; step < 125 && !found; ++step) {
					const Standard_Integer offsets[3] = {
						step % 5 - 2, (step / 5) % 5 - 2, step / 25 - 2};
					if (std::max({std::abs(offsets[0]), std::abs(offsets[1]),
							std::abs(offsets[2])}) != reach) continue;
					Standard_Real moved[3];
					for (Standard_Integer axis = 0; axis < 3; ++axis) {
						moved[axis] = base[axis];
						for (Standard_Integer k = 0; k < std::abs(offsets[axis]); ++k) {
							moved[axis] = std::nextafter(moved[axis],
								offsets[axis] > 0 ? 2.0 : -2.0);
						}
					}
					try {
						gp_Ax3 trial(read.Location(), n,
							gp_Dir(moved[0], moved[1], moved[2]));
						if (trial.Direct() != read.Direct()) trial.YReverse();
						if (PlaneFrameIsFileStable(trial)
							&& trial.XDirection().Angle(read.XDirection()) < 1e-12) {
							plane->SetPosition(trial);
							found = Standard_True;
						}
					} catch (...) {}
				}
			}
			if (!found) ++unsettled;
		}
		return unsettled;
	}
	} // namespace plane_frames
} // namespace core3d
