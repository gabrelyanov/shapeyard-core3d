#pragma once
#include "SpatialSweepAdmission.hxx"
#include "SpatialSweepGeomFillBuilder.hxx"
#include "SpatialSweepPersistence.hxx"
#include "SpatialSweepRebuild.hxx"
#include <map>
#include <string>

namespace core3d::spatial_sweep::debug {
inline UUID ID(std::uint8_t value) {
    UUID result{}; result.front() = value; return result;
}
inline Digest Hash(std::uint8_t value) {
    Digest result{}; result.front() = value; return result;
}

inline bounded_curve::Definition Curve() {
    bounded_curve::Definition value;
    value.domain = bounded_curve::Domain::Path3D;
    value.frame.identifier = ID(10); value.frame.revision = 1;
    value.degree = 1;
    value.controlPoints = {{{ID(11), {{0, 0, 0}}}, {ID(12), {{100, 0, 0}}}}};
    value.knots = {{0, 2}, {1, 2}};
    return value;
}

inline Definition Sweep() {
    Definition value;
    value.path.inputNode = ID(20); value.path.curveFeature = ID(21);
    value.path.sourceRecipeDigest = Hash(22);
    value.path.ownerState.definitionRevision = 3;
    value.path.ownerState.nextLocalID = 4;
    value.path.ownerState.canonicalDefinitionDigest = Hash(23);
    value.dimensionMetersPerUnit = 0.001;
    value.sectionIdentifier = ID(24);
    value.orientation.authoredSeed = {{0, 0, 1}};
    value.radius = {RadiusLawKind::Constant, 2, 2};
    value.twist = {TwistLawKind::LinearArcLength, 0, 0};
    return value;
}

inline CurveCertificate Proof() {
    CurveCertificate value;
    value.lengthMM = {100, 100};
    value.speedPerNormalizedParameter = {100, 100};
    value.curvaturePerMM = {0, 0};
    value.maximumAbsCoordinateMM = {100, 100};
    value.nonlocalCentrelineDistanceMM = {100, 100};
    value.holonomyRadians = {0, 0};
    value.homogeneousBernsteinBounds = true;
    value.everyJoinExactG1G2 = true;
    value.localBandInjective = true;
    value.fullPairDomainVisited = true;
    value.work = {8, 4, 16, 0, 512};
    value.provenance = CurveProofProvenance::None; // Deliberately cannot admit.
    return value;
}

inline bool Near(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}

inline bounded_curve::Definition InflectionCurve() {
    auto value=Curve();value.degree=3;value.controlPoints={{{ID(31),{{-120,-24,0}}},{ID(32),{{-40,24,0}}},{ID(33),{{40,-24,0}}},{ID(34),{{120,24,0}}}}};value.knots={{-1,4},{1,4}};return value;
}
inline bounded_curve::Definition ContactCurve() {
    auto value=Curve();value.degree=3;value.controlPoints={{{ID(41),{{0,0,0}}},{ID(42),{{100,0,0}}},{ID(43),{{100,1,0}}},{ID(44),{{0,1,0}}}}};value.knots={{0,4},{1,4}};return value;
}
inline bounded_curve::Definition CircleCurve() {
    auto value=Curve();value.degree=2;value.controlPoints={{{ID(51),{{100,0,0}}},{ID(52),{{100,100,0}}},{ID(53),{{0,100,0}}},{ID(54),{{-100,100,0}}},{ID(55),{{-100,0,0}}},{ID(56),{{-100,-100,0}}},{ID(57),{{0,-100,0}}},{ID(58),{{100,-100,0}}},{ID(59),{{100,0,0}}}}};value.knots={{0,3},{0.25,2},{0.5,2},{0.75,2},{1,3}};const double w=std::sqrt(0.5);value.weights={1,w,1,w,1,w,1,w,1};return value;
}
inline bounded_curve::Definition SpatialPolynomialCurve() {
    auto value=Curve();value.degree=3;value.controlPoints={{{ID(61),{{0,0,0}}},{ID(62),{{40,0,0}}},{ID(63),{{80,10,0}}},{ID(64),{{120,30,30}}}}};value.knots={{0,4},{1,4}};return value;
}
inline const std::map<std::string,bool>& RealKernelChecks() {
    static const std::map<std::string,bool> cached=[] {
    std::map<std::string,bool> checks;std::atomic_bool cancelled{false};auto sweep=Sweep();const double R=2+4*PositionalEpsilonMM;
    const auto straight=proof_producer::Produce(Curve(),1,R,false);checks["producer-straight-receipt"]=straight.produced()&&straight.certificate.homogeneousBernsteinBounds&&straight.certificate.fullPairDomainVisited;
    auto zero=Curve();zero.controlPoints.back().local=zero.controlPoints.front().local;const auto zeroProof=proof_producer::Produce(zero,1,R,false);checks["producer-zero-tangent-refuses"]=!zeroProof.produced();
    const auto inflection=InflectionCurve();const auto inflectionProof=proof_producer::Produce(inflection,1,R,false);checks["producer-inflection-receipt"]=inflectionProof.produced();
    try {Handle(Geom_BSplineCurve) geometry=geomfill_detail::Curve(inflection,1);Handle(GeomAdaptor_Curve) adaptor=new GeomAdaptor_Curve(geometry);auto table=PrepareBishopTransport(adaptor,{{0,0,1}},0.2,1.1,FrameTolerance(R),&cancelled);Handle(BishopTrihedronLaw) law=table?new BishopTrihedronLaw(table,0.2,1.1):nullptr;gp_Vec t,n,b,dt,dn,db,d2t,d2n,d2b;bool callbacks=!law.IsNull()&&law->SetCurve(adaptor)&&law->D2(0.5,t,dt,d2t,n,dn,d2n,b,db,d2b)&&law->D0(-0.5,t,n,b);Handle(GeomFill_TrihedronLaw) copy=law.IsNull()?Handle(GeomFill_TrihedronLaw)():law->Copy();if(!copy.IsNull())copy->SetInterval(-0.75,0.75);checks["bishop-law-copy-interval-d1-d2"]=callbacks&&!copy.IsNull()&&copy->D1(0,t,dt,n,dn,b,db)&&n.Z()>0;checks["bishop-law-query-order-independent"]=callbacks&&law->D0(0.5,t,n,b)&&n.Z()>0;}catch(...){checks["bishop-law-copy-interval-d1-d2"]=false;checks["bishop-law-query-order-independent"]=false;}
    const auto contact=proof_producer::Produce(ContactCurve(),1,R,false);checks["producer-return-contact-refuses"]=contact.status==proof_producer::Status::NonlocalContact||contact.status==proof_producer::Status::NonlocalClearanceUnproved;
    const auto circle=CircleCurve();const auto circleProof=proof_producer::Produce(circle,1,5+4*PositionalEpsilonMM,true);checks["producer-circle-curvature-bound"]=circleProof.produced()&&circleProof.certificate.endpointsExactlyEqual&&circleProof.certificate.seamExactG1G2&&circleProof.certificate.curvaturePerMM.upper>0.0099;
    try{Handle(Geom_BSplineCurve) circleGeometry=geomfill_detail::Curve(circle,1);Handle(GeomAdaptor_Curve) circleAdaptor=new GeomAdaptor_Curve(circleGeometry);auto circleTable=PrepareBishopTransport(circleAdaptor,{{0,0,1}},0,0,FrameTolerance(5+4*PositionalEpsilonMM),&cancelled);double h=0;checks["bishop-closed-circle-holonomy"]=circleTable&&Holonomy(circleTable->nodes.front().bishop,circleTable->nodes.back().bishop,h)&&std::abs(h)<1e-7;}catch(...){checks["bishop-closed-circle-holonomy"]=false;}
    auto built=BuildGeomFillSweep(Curve(),sweep,cancelled);checks["geomfill-real-cylinder-built"]=built.built()&&!built.solid.IsNull();checks["geomfill-real-returned-surfaces-certified"]=built.built()&&built.receipt.surface.cells.size()==4&&ValidateSurfaceProof(built.receipt.surface,built.receipt.independentMeasure,sweep.closure)==SurfaceProofRefusal::None;checks["geomfill-real-solid-gated"]=built.built()&&built.receipt.topology.selfInterferenceCheckedSerially&&built.receipt.topology.selfInterferenceFree;
    auto closedSweep=sweep;closedSweep.radius={RadiusLawKind::Constant,5,5};closedSweep.closure=ClosureKind::ClosedNoCaps;closedSweep.twist={TwistLawKind::CloseFrame,0,0};closedSweep.witness.present=true;closedSweep.witness.unwrappedHolonomyReference=0;auto closedBuilt=BuildGeomFillSweep(circle,closedSweep,cancelled);checks["geomfill-real-closed-built"]=closedBuilt.built()&&closedBuilt.receipt.surface.closedHasNoCaps;
    auto polynomialSweep=sweep;polynomialSweep.radius={RadiusLawKind::LinearArcLength,2,3};auto polynomialBuilt=BuildGeomFillSweep(SpatialPolynomialCurve(),polynomialSweep,cancelled);checks["geomfill-real-spatial-polynomial-built"]=polynomialBuilt.built()&&polynomialBuilt.receipt.surface.provenance==SurfaceCertificate::Provenance::ReturnedRationalSurfaceIntervalsV1;
    checks["producer-budgets-exact"]=straight.produced()&&WithinBudget(straight.certificate.work)&&straight.certificate.work.exactArithmeticBits<=MaximumExactArithmeticBits;
    checks["producer-curvature-whole-span"]=inflectionProof.produced()&&inflectionProof.certificate.curvaturePerMM.upper>0;
    checks["producer-closed-seam-is-exact-only"]=!straight.certificate.endpointsExactlyEqual&&!straight.certificate.seamExactG1G2;
    // Controls for the demonstrated K1b defects. Carry widening is checked
    // against independently structured values, including propagated carries,
    // negative sums, cancellation, multiplication/division consistency and
    // the exact 4096-bit boundary; every existing budget refusal stays.
    {bool carry=true;try{
        using proof_producer::detail::cpp_int;using proof_producer::detail::Budget;
        auto shifted=[](std::int64_t base,unsigned bits){cpp_int out(base);out<<=bits;return out;};
        const cpp_int max32(std::int64_t(0xffffffff));
        carry=carry&&(max32+cpp_int(1)==shifted(1,32));
        carry=carry&&(max32+max32==shifted(1,33)-cpp_int(2));
        const cpp_int high=shifted(1,96);carry=carry&&((high-cpp_int(1))+cpp_int(1)==high);
        carry=carry&&((-max32)+(-cpp_int(1))==-shifted(1,32));
        const cpp_int wide=shifted(1,200)-cpp_int(12345);carry=carry&&(wide+(-wide)==cpp_int(0)&&wide-wide==cpp_int(0));
        const cpp_int a=shifted(1,100)+cpp_int(12345),b(std::int64_t(0xffffff));carry=carry&&((a*b)/b==a&&(a*b)%b==cpp_int(0));
        carry=carry&&shifted(1,4095).bitCount()==4096;
        bool refused=false;try{const cpp_int boundary=shifted(1,4095);(void)(boundary+boundary);}catch(const Budget&){refused=true;}carry=carry&&refused;
    }catch(...){carry=false;}checks["producer-exact-carry-controls"]=carry;}
    // Down/Up must enclose the exact rational provably: the candidate's own
    // persisted bits are compared exactly, and nonfinite conversions refuse.
    {bool outward=true;try{
        using proof_producer::detail::cpp_int;using proof_producer::detail::Rational;using proof_producer::detail::Budget;using proof_producer::detail::Down;using proof_producer::detail::Up;
        auto encloses=[&](const Rational& r){const double lo=Down(r),hi=Up(r);return std::isfinite(lo)&&std::isfinite(hi)&&!(r<Rational::FromDouble(lo))&&!(Rational::FromDouble(hi)<r);};
        outward=outward&&encloses(Rational(cpp_int(1),cpp_int(10)))&&encloses(Rational(cpp_int(-1),cpp_int(10)));
        cpp_int big(1);big<<=400;outward=outward&&encloses(Rational(big,big-cpp_int(1)))&&encloses(Rational(cpp_int(-1),big));
        bool refused=false;try{cpp_int huge(1);huge<<=4095;(void)Down(Rational(huge,cpp_int(1)));}catch(const Budget&){refused=true;}outward=outward&&refused;
    }catch(...){outward=false;}checks["producer-rational-enclosure-outward"]=outward;}
    // Copied laws carry initialized OCCT base state: GetInterval, direct and
    // nested copied SetInterval and derivative callbacks work without
    // reseeding transport; a coincident-bounds foreign curve is refused.
    try{Handle(Geom_BSplineCurve) geometry=geomfill_detail::Curve(inflection,1);Handle(GeomAdaptor_Curve) adaptor=new GeomAdaptor_Curve(geometry);auto table=PrepareBishopTransport(adaptor,{{0,0,1}},0.2,1.1,FrameTolerance(R),&cancelled);Handle(BishopTrihedronLaw) law=table?new BishopTrihedronLaw(table,0.2,1.1):nullptr;gp_Vec t,n,b,dt,dn,db,d2t,d2n,d2b;bool ok=!law.IsNull();Handle(GeomFill_TrihedronLaw) copy=ok?law->Copy():Handle(GeomFill_TrihedronLaw)();if(copy.IsNull())ok=false;else{copy->SetInterval(-0.75,0.75);Standard_Real cf=0,cl=0;copy->GetInterval(cf,cl);ok=ok&&Near(cf,-0.75)&&Near(cl,0.75)&&copy->D1(0,t,dt,n,dn,b,db);Handle(GeomFill_TrihedronLaw) nested=copy->Copy();if(nested.IsNull())ok=false;else{nested->SetInterval(-0.5,0.5);ok=ok&&nested->D2(0.25,t,dt,d2t,n,dn,d2n,b,db,d2b);}auto moved=InflectionCurve();moved.controlPoints[1].local={-40,30,0};Handle(GeomAdaptor_Curve) foreign=new GeomAdaptor_Curve(geomfill_detail::Curve(moved,1));Handle(GeomAdaptor_Curve) rewrap=new GeomAdaptor_Curve(geometry);ok=ok&&law->SetCurve(rewrap)&&!law->SetCurve(foreign);}checks["bishop-law-copy-base-initialized"]=ok;}catch(...){checks["bishop-law-copy-base-initialized"]=false;}
    // Clipped interval counts agree with the emitted breaks on a multispan
    // trim of the circle, including nested trims and endpoint queries.
    try{Handle(Geom_BSplineCurve) circleGeometry=geomfill_detail::Curve(circle,1);Handle(GeomAdaptor_Curve) circleAdaptor=new GeomAdaptor_Curve(circleGeometry);auto circleTable=PrepareBishopTransport(circleAdaptor,{{0,0,1}},0,0,FrameTolerance(5+4*PositionalEpsilonMM),&cancelled);Handle(BishopTrihedronLaw) law=circleTable?new BishopTrihedronLaw(circleTable,0,0):nullptr;gp_Vec t,n,b;bool ok=!law.IsNull();Handle(GeomFill_TrihedronLaw) trimmed=ok?law->Copy():Handle(GeomFill_TrihedronLaw)();if(trimmed.IsNull())ok=false;else{trimmed->SetInterval(0.1,0.9);const Standard_Integer count=trimmed->NbIntervals(GeomAbs_C2);TColStd_Array1OfReal knots(1,count+1);trimmed->Intervals(knots,GeomAbs_C2);ok=ok&&count==4&&Near(knots(1),0.1)&&Near(knots(2),0.25)&&Near(knots(3),0.5)&&Near(knots(4),0.75)&&Near(knots(5),0.9);ok=ok&&trimmed->D0(0.5,t,n,b)&&!trimmed->D0(0.05,t,n,b)&&trimmed->D0(std::nextafter(0.9,std::numeric_limits<double>::infinity()),t,n,b);Handle(GeomFill_TrihedronLaw) nested=trimmed->Copy();if(nested.IsNull())ok=false;else{nested->SetInterval(0.3,0.7);ok=ok&&nested->NbIntervals(GeomAbs_C2)==2&&nested->D0(0.5,t,n,b)&&!nested->D0(0.75,t,n,b);}}checks["bishop-law-trim-multispan-intervals"]=ok;}catch(...){checks["bishop-law-trim-multispan-intervals"]=false;}
    // Exactly-one-ULP endpoint requests normalize to the retained endpoint
    // for value and both derivatives; material excursions still return NaN.
    try{Handle(Geom_BSplineCurve) geometry=geomfill_detail::Curve(Curve(),1);Handle(GeomAdaptor_Curve) adaptor=new GeomAdaptor_Curve(geometry);auto table=PrepareBishopTransport(adaptor,{{0,0,1}},0,0,FrameTolerance(R),&cancelled);Handle(ArcLengthRadiusLaw) law=table?new ArcLengthRadiusLaw(table,2,3):nullptr;const double below=std::nextafter(0.0,-std::numeric_limits<double>::infinity()),above=std::nextafter(1.0,std::numeric_limits<double>::infinity());Standard_Real f=0,d=0,d2=0;bool ok=!law.IsNull();if(ok){law->D1(above,f,d);ok=Near(f,3)&&std::isfinite(d);law->D2(below,f,d,d2);ok=ok&&Near(f,2)&&std::isfinite(d)&&std::isfinite(d2);ok=ok&&Near(law->Value(below),2)&&Near(law->Value(above),3)&&Near(law->Value(0),2)&&Near(law->Value(1),3)&&!std::isfinite(law->Value(1.0+1e-6))&&!std::isfinite(law->Value(-1e-6));}checks["radius-law-endpoint-one-ulp-normalized"]=ok;}catch(...){checks["radius-law-endpoint-one-ulp-normalized"]=false;}
    return checks;
    }();
    return cached;
}

inline DetachedKernelReceipt Candidate(double startRadiusMM = 5,
                                       double endRadiusMM = 5) {
    DetachedKernelReceipt value;
    value.surface.cells = {{{-1.1, -0.9}, {-0.0002, 0.0002}, {0.5, 0.7}, true, true}};
    value.surface.orderedQuarterCoverage = {{true, true, true, true}};
    value.surface.sharedMeridiansMatch = true;
    value.surface.pathBoundaryCoveredOnce = true;
    value.surface.openCapsPlanarAndOriented = true;
    value.surface.fittingErrorMM = 0.0002;
    value.surface.transportErrorMM = 0.0002;
    value.surface.sewingErrorMM = 0.0002;
    value.surface.serializationErrorMM = 0.0002;
    value.surface.work = {16, 5, 32, 1, 1024};
    value.surface.provenance = SurfaceCertificate::Provenance::SyntheticFixture;
    const double volume = IdealCircularSweepVolume(100, startRadiusMM, endRadiusMM);
    value.independentMeasure.sourceLengthMM = {100, 100};
    value.independentMeasure.expectedVolumeMM3 = {volume - 1e-8, volume + 1e-8};
    value.independentMeasure.measuredVolumeMM3 = {volume, volume};
    for (int station = 0; station <= 4; ++station) {
        const double radius = startRadiusMM
            + (endRadiusMM - startRadiusMM) * (double(station) / 4);
        value.independentMeasure.stationRadiusMM.push_back({radius, radius});
    }
    value.independentMeasure.sourcePolynomialIntegratedIndependently = true;
    value.independentMeasure.everyLocalSectionAccountedFor = true;
    value.independentMeasure.markedMeridianMatches = true;
    value.topology = {1, 1, 6, 12, true, true, true, true, true, true,
                      true, true, true};
    value.geometryBinaryDigest = Hash(61);
    value.serializerFormatVersion = 4;
    value.usedBishopTrihedron = true;
    value.usedFourKnownQuarterSections = true;
    value.withKpartDisabled = true;
    value.forceApproxC1Disabled = true;
    value.deterministicSpanOrder = true;
    return value;
}

inline bool NormalizeFixture(const GeometryBytes& input, GeometryBytes& output) noexcept {
    output = input;
    if (output.empty()) return false;
    if (output.front() == 0xa0) output.front() = 0xc0;
    else if (output.front() == 0xd0) output.front() = 0xe0;
    else if (output.front() == 0xe0) output.front() = 0xd0;
    return true;
}

inline RetainedIdentityFence Fence() {
    RetainedIdentityFence value;
    value.owner = ID(70); value.definition = ID(71); value.sourceNode = ID(72);
    value.curveFeature = ID(73); value.sweepFeature = ID(74); value.section = ID(75);
    value.ownerRevision = 8; value.curveRevision = 3;
    value.sourceDigest = Hash(76); value.featureDigest = Hash(77);
    return value;
}

inline std::map<std::string, bool> Probe() {
    std::map<std::string, bool> checks;
    Definition sweep = Sweep();
    const bounded_curve::Definition curve = Curve();
    const auto producedProof = proof_producer::Produce(curve, 1, 2 + 4 * PositionalEpsilonMM, false);
    CurveCertificate proof = producedProof.certificate;

    std::vector<std::uint8_t> payload;
    Definition decoded;
    checks["k0-value-valid"] = Validate(sweep) == Refusal::None;
    checks["k0-canonical-roundtrip"] = Encode(sweep, payload) && Decode(payload, decoded)
        && SameDefinition(sweep, decoded) && payload.size() <= MaximumPayloadBytes;
    auto trailing = payload; trailing.push_back(0);
    checks["k0-trailing-refused"] = !Decode(trailing, decoded);
    auto malformed = payload; if (!malformed.empty()) malformed[0] ^= 1;
    checks["k0-malformed-refused"] = !Decode(malformed, decoded);
    checks["k0-carrier-remains-uninstalled"] = ValidateCarrierFeature(payload,
        sweep.path.inputNode, SpatialCircleSweepFeatureKind,
        SpatialCircleSweepFeatureCodec) == Refusal::RuleNotInstalled;
    checks["k0-wrong-arity-binding-refused"] = ValidateCarrierFeature(payload,
        ID(99), SpatialCircleSweepFeatureKind,
        SpatialCircleSweepFeatureCodec) == Refusal::NonCanonicalEncoding;
    checks["k0-budgets-frozen"] = MaximumPayloadBytes == 16 * 1024
        && MaximumCurveDefinitionBytes == 16 * 1024
        && MaximumCurveDocumentBytes == 1024 * 1024
        && MaximumCurveRecordsPerDocument == 256;

    const auto admitted = Admit(sweep, curve, proof, {{1, 0, 0}}, true);
    checks["regular-straight-admits"] = admitted.admitted();
    auto zero = proof; zero.speedPerNormalizedParameter.lower = 0;
    checks["zero-tangent-refused"] = Admit(sweep, curve, zero, {{1, 0, 0}}, true).refusal
        == AdmissionRefusal::ZeroTangent;
    auto kink = proof; kink.everyJoinExactG1G2 = false;
    checks["kink-refused"] = Admit(sweep, curve, kink, {{1, 0, 0}}, true).refusal
        == AdmissionRefusal::Kink;
    auto exhausted = proof; exhausted.work.proofLeaves = MaximumProofLeaves + 1;
    checks["certificate-exhaustion-refused"] = Admit(sweep, curve, exhausted,
        {{1, 0, 0}}, true).refusal == AdmissionRefusal::CertificationBudgetExceeded;

    Frame frame{};
    bool inflection = SeedFrame({{1, 1, 0}}, {{0, 0, 1}}, frame);
    for (const Vector3 tangent : {Vector3{{1, 0.4, 0}}, Vector3{{1, 0, 0}},
                                  Vector3{{1, -0.4, 0}}, Vector3{{1, -1, 0}}}) {
        Frame next{}; inflection = inflection && BishopStep(frame, tangent, next)
            && Dot(next.e1, Vector3{{0, 0, 1}}) > 1 - 1e-10;
        frame = next;
    }
    checks["bishop-inflection-no-flip"] = inflection;
    Frame straightStart{}, straightEnd{};
    checks["straight-interval-keeps-seed"] = SeedFrame({{1, 0, 0}}, {{0, 0, 1}}, straightStart)
        && BishopStep(straightStart, {{1, 0, 0}}, straightEnd)
        && Dot(straightStart.e1, straightEnd.e1) > 1 - 1e-12;
    checks["frame-right-handed"] = ValidFrame(frame);

    const double d = std::sqrt(11600.0), length = 2 * Pi * d;
    const double torsion = 40.0 / 11600.0;
    const double expectedBishopAngle = torsion * length;
    checks["helix-closed-form-finite"] = Near(expectedBishopAngle,
        80 * Pi / std::sqrt(11600.0), 1e-12);
    PreparedTransport table;
    table.stations = {{0, straightStart}, {1, straightEnd}};
    table.phase = 0.2; table.totalSpin = 1.1;
    Frame queryA{}, queryB{}, queryAgain{};
    const bool queryOrder = Evaluate(table, 0.75, {{1, 0, 0}}, queryA)
        && Evaluate(table, 0.25, {{1, 0, 0}}, queryB)
        && Evaluate(table, 0.75, {{1, 0, 0}}, queryAgain);
    checks["transport-query-order-invariant"] = queryOrder
        && Dot(queryA.e1, queryAgain.e1) > 1 - 1e-12;
    checks["transport-integrated-spin"] = queryOrder
        && Near(std::atan2(queryA.e1[1], queryA.e1[2]), -(0.2 + 1.1 * 0.75), 1e-9);

    Definition closed = sweep;
    closed.closure = ClosureKind::ClosedNoCaps;
    closed.twist = {TwistLawKind::CloseFrame, 0, 0};
    closed.witness.present = true;
    closed.witness.unwrappedHolonomyReference = 0.4;
    auto closedProof = proof;
    closedProof.lengthMM = {200, 200}; closedProof.speedPerNormalizedParameter = {200, 200};
    closedProof.endpointsExactlyEqual = true; closedProof.seamExactG1G2 = true;
    closedProof.holonomyRadians = {0.4, 0.4};
    const auto closedAdmission = Admit(closed, curve, closedProof, {{1, 0, 0}}, false);
    checks["closed-holonomy-corrected"] = closedAdmission.admitted()
        && Near(0.4 + closedAdmission.correctedTotalSpin, 0);
    double lifted = 0; std::int32_t lift = 0;
    checks["closed-branch-reference-stable"] = SelectHolonomyLift(-Pi + 1e-6,
        Pi + 1e-6, false, 1e-10, lifted, lift) && lift == 1;
    checks["closed-half-turn-ambiguous"] = !SelectHolonomyLift(Pi, 0, true,
        1e-10, lifted, lift);
    auto openSeam = closedProof; openSeam.seamExactG1G2 = false;
    checks["closed-open-seam-refused"] = Admit(closed, curve, openSeam,
        {{1, 0, 0}}, false).refusal == AdmissionRefusal::OpenSeam;

    auto below = proof;
    below.curvaturePerMM = {0, std::nextafter(0.25 / admitted.radiusEnvelopeMM, 0.0)};
    checks["curvature-below-bound-admits"] = Admit(sweep, curve, below,
        {{1, 0, 0}}, true).admitted();
    auto equality = proof;
    equality.curvaturePerMM = {0, 0.25 / admitted.radiusEnvelopeMM};
    checks["curvature-equality-refused"] = Admit(sweep, curve, equality,
        {{1, 0, 0}}, true).refusal == AdmissionRefusal::TightCurvature;

    const double radiusEnvelope = admitted.radiusEnvelopeMM;
    PairCell straddling{{4 * radiusEnvelope - 0.01, 4 * radiusEnvelope + 0.01},
                        {0, 4 * radiusEnvelope}};
    checks["nonlocal-straddling-cell-must-split"] =
        ClassifyPairCell(straddling, radiusEnvelope) == PairDecision::MustSplit;
    auto contact = proof;
    contact.nonlocalCentrelineDistanceMM = {0, 2 * radiusEnvelope};
    checks["nonlocal-contact-refused"] = Admit(sweep, curve, contact,
        {{1, 0, 0}}, true).refusal == AdmissionRefusal::NonlocalContact;
    auto incomplete = proof; incomplete.fullPairDomainVisited = false;
    checks["nonlocal-incomplete-refused"] = Admit(sweep, curve, incomplete,
        {{1, 0, 0}}, true).refusal == AdmissionRefusal::NonlocalClearanceUnproved;

    RadiusLaw linear{RadiusLawKind::LinearArcLength, 5, 8};
    LawEvaluation law{};
    checks["arc-length-law-chain-rule"] = EvaluateRadiusLaw(linear, 40, 100, 3, 2, law)
        && Near(law.radius, 6.2) && Near(law.firstDerivative, 0.09)
        && Near(law.secondDerivative, 0.06);
    auto ratio = sweep; ratio.radius = {RadiusLawKind::LinearArcLength, 1, 4.0000001};
    checks["radius-ratio-not-clamped"] = Admit(ratio, curve, proof,
        {{1, 0, 0}}, true).refusal == AdmissionRefusal::LawOutOfBounds;
    auto excessiveTwist = sweep; excessiveTwist.twist.totalRadians = 4 * Pi + 1e-12;
    checks["twist-not-clamped"] = Admit(excessiveTwist, curve, proof,
        {{1, 0, 0}}, true).refusal == AdmissionRefusal::LawOutOfBounds;
    auto parallelSeed = sweep; parallelSeed.orientation.authoredSeed = {{1, 0, 0}};
    checks["parallel-seed-refused"] = Admit(parallelSeed, curve, proof,
        {{1, 0, 0}}, true).refusal == AdmissionRefusal::ParallelSeed;
    auto closedLinear = closed; closedLinear.radius = {RadiusLawKind::LinearArcLength, 2, 3};
    checks["closed-linear-law-refused"] = Validate(closedLinear) == Refusal::BadTwistLaw;
    const auto& real=RealKernelChecks();checks.insert(real.begin(),real.end());return checks;
}

inline std::map<std::string, bool> K2Probe() {
    std::map<std::string, bool> checks;
    Definition sweep = Sweep();
    std::atomic_bool cancelled{false};
    const auto realCylinder = BuildGeomFillSweep(Curve(), sweep, cancelled);
    const auto admission = realCylinder.admission;

    checks["analytic-cylinder-volume"] = Near(IdealCircularSweepVolume(100, 5, 5),
        2500 * Pi, 1e-9);
    checks["analytic-taper-volume"] = Near(IdealCircularSweepVolume(100, 5, 8),
        4300 * Pi, 1e-9);
    auto taperDefinition=sweep;taperDefinition.radius={RadiusLawKind::LinearArcLength,5,8};
    const auto realTaper=BuildGeomFillSweep(Curve(),taperDefinition,cancelled);
    const auto& taper=realTaper.receipt;
    // FIX-SPEC step 4: station radii are now measured on the returned
    // surfaces, so the receipt carries honest intervals. Each interval must
    // enclose the independently authored expectation within the frozen
    // positional tolerance; the authored expression is the expectation a
    // measurement is compared against, never the measurement of its own
    // result.
    auto stationsEnclose=[](const std::vector<Interval>& measured,double startRadius,double endRadius){
        if(measured.size()!=5)return false;
        for(int index=0;index<=4;++index){
            const double expected=startRadius+(endRadius-startRadius)*index/4;
            const Interval& value=measured[std::size_t(index)];
            if(!Valid(value)||value.lower<=0||value.lower>expected||value.upper<expected||value.upper-value.lower>PositionalEpsilonMM)return false;
        }
        return true;
    };
    checks["analytic-station-radii"] = realTaper.built()
        && stationsEnclose(taper.independentMeasure.stationRadiusMM,5,8);
    // Geometry-level negative: radii measured on one real build must not
    // satisfy the expectation of a different authored taper.
    checks["measure-foreign-taper-stations-refused"] = realTaper.built() && realCylinder.built()
        && !stationsEnclose(realCylinder.receipt.independentMeasure.stationRadiusMM,5,8)
        && !stationsEnclose(taper.independentMeasure.stationRadiusMM,5,5);
    checks["marked-meridian-and-cap-signs"] = taper.independentMeasure.markedMeridianMatches
        && taper.surface.openCapsPlanarAndOriented && !taper.surface.closedHasNoCaps;

    auto candidate = realCylinder.receipt;
    // An invalid positive fixture must fail every dependent check explicitly;
    // it must never be counted as successful exercise of a refusal path.
    const bool validPositive = realCylinder.built()
        && candidate.surface.provenance == SurfaceCertificate::Provenance::ReturnedRationalSurfaceIntervalsV1
        && candidate.surface.cells.size() == 4;
    checks["surface-four-quarter-map-admits"] = validPositive
        && ValidateDetachedSolid(sweep, admission, candidate).admitted();
    auto wrongSpine = candidate;
    checks["surface-wrong-spine-refused"] = validPositive && !wrongSpine.surface.cells.empty();
    if (!wrongSpine.surface.cells.empty()) {
        wrongSpine.surface.cells.front().otherSpineIntervalsDischarged = false;
        checks["surface-wrong-spine-refused"] = validPositive && ValidateDetachedSolid(sweep, admission,
            wrongSpine).refusal == SolidRefusal::SurfaceProof;
    }
    auto reversedTaper = candidate;
    reversedTaper.independentMeasure.measuredVolumeMM3 = {1, 1};
    checks["surface-reversed-taper-refused"] = validPositive && ValidateDetachedSolid(sweep, admission,
        reversedTaper).refusal == SolidRefusal::SurfaceProof;
    auto collapsedQuarter = candidate;
    collapsedQuarter.surface.orderedQuarterCoverage[2] = false;
    checks["surface-collapsed-quarter-refused"] = validPositive && ValidateDetachedSolid(sweep, admission,
        collapsedQuarter).refusal == SolidRefusal::SurfaceProof;
    // Independently constructed negatives for the measured quantities: an
    // inside-out normal field, an excessive radial deviation and a
    // degenerate oriented Jacobian are exactly what wrong returned geometry
    // would measure, and each must refuse through the unchanged gate.
    auto insideOut = candidate;
    for (auto& cell : insideOut.surface.cells) cell.normalEquationDerivative = {0.9, 1.1};
    checks["surface-inside-out-normal-refused"] = validPositive
        && ValidateDetachedSolid(sweep, admission, insideOut).refusal == SolidRefusal::SurfaceProof;
    auto radialOff = candidate;
    for (auto& cell : radialOff.surface.cells) cell.radialDeviationMM = {-2 * PositionalEpsilonMM, 2 * PositionalEpsilonMM};
    checks["surface-excess-radial-deviation-refused"] = validPositive
        && ValidateDetachedSolid(sweep, admission, radialOff).refusal == SolidRefusal::SurfaceProof;
    auto degenerate = candidate;
    for (auto& cell : degenerate.surface.cells) cell.orientedMapJacobian = {-0.5, 0.5};
    checks["surface-degenerate-jacobian-refused"] = validPositive
        && ValidateDetachedSolid(sweep, admission, degenerate).refusal == SolidRefusal::SurfaceProof;

    auto interference = candidate; interference.topology.selfInterferenceFree = false;
    checks["solid-interference-refused"] = validPositive && ValidateDetachedSolid(sweep, admission,
        interference).refusal == SolidRefusal::SelfInterference;
    auto openShell = candidate; openShell.topology.sewingHasNoFreeOrMultipleEdges = false;
    checks["solid-open-shell-refused"] = validPositive && ValidateDetachedSolid(sweep, admission,
        openShell).refusal == SolidRefusal::OpenShell;
    auto extraSolid = candidate; extraSolid.topology.solidCount = 2;
    checks["solid-extra-solid-refused"] = validPositive && ValidateDetachedSolid(sweep, admission,
        extraSolid).refusal == SolidRefusal::ExtraSolid;
    auto flipped = candidate; flipped.topology.forwardOriented = false;
    checks["solid-flipped-face-refused"] = validPositive && ValidateDetachedSolid(sweep, admission,
        flipped).refusal == SolidRefusal::InvalidSolid;

    const GeometryBytes millimetres{0xa0, 1, 2, 3};
    const GeometryBytes metres{0xa0, 4, 5, 6};
    checks["fixed-point-millimetres"] = PrepareFixedPoint(millimetres, millimetres,
        NormalizeFixture).admitted();
    checks["fixed-point-metres"] = PrepareFixedPoint(metres, metres,
        NormalizeFixture).admitted();
    checks["fixed-point-drift-refused"] = PrepareFixedPoint({0xd0, 1}, {0xd0, 1},
        NormalizeFixture).refusal == FixedPointRefusal::NonFixedPoint;
    checks["independent-build-mismatch-refused"] = PrepareFixedPoint(
        {0xa0, 1}, {0xa0, 2}, NormalizeFixture).refusal
        == FixedPointRefusal::IndependentBuildMismatch;
    BuildWitnessReceipt witness;
    witness.profile = sweep.profile; witness.binaryFormatVersion = 4;
    witness.geometryBinaryDigest = Hash(81); witness.surfaceCertificateDigest = Hash(82);
    checks["build-witness-profile-bound"] = ValidateBuildWitness(sweep, witness);

    std::vector<std::uint8_t> legacyBytes{'S', 'W', 'P', 'R', 1, 0, 0, 0};
    checks["legacy-bytes-not-synthesized-as-scsw"] = ValidateCarrierFeature(legacyBytes,
        sweep.path.inputNode, SpatialCircleSweepFeatureKind,
        SpatialCircleSweepFeatureCodec) == Refusal::NonCanonicalEncoding;
    checks["legacy-route-remains-uninstalled"] = ValidateCarrierFeature({},
        sweep.path.inputNode, SpatialCircleSweepFeatureKind,
        SpatialCircleSweepFeatureCodec) == Refusal::NonCanonicalEncoding;

    ReplayPreparation replay;
    replay.captured = Fence(); replay.reread = replay.captured;
    replay.allDescendantsSupported = true; replay.detachedCandidateAdmitted = true;
    replay.stageWouldSucceed = true; replay.bindingVerificationWouldSucceed = true;
    checks["composite-missing-c1-codec-refuses-before-command"] =
        PrepareRetainedReplay(replay).refusal == ReplayRefusal::MissingCanonicalRecipe
        && !PrepareRetainedReplay(replay).mayOpenOneOwnedCommand;
    replay.canonicalC1AndCompositeBytesAvailable = true;
    auto foreign = replay; foreign.reread.sourceDigest = Hash(99);
    checks["composite-foreign-source-refuses-atomically"] =
        PrepareRetainedReplay(foreign).refusal == ReplayRefusal::Stale
        && PrepareRetainedReplay(foreign).oldStateMustRemainExact;
    auto unsupported = replay; unsupported.allDescendantsSupported = false;
    checks["composite-unsupported-descendant-refuses-atomically"] =
        PrepareRetainedReplay(unsupported).refusal == ReplayRefusal::UnsupportedDependent
        && !PrepareRetainedReplay(unsupported).mayOpenOneOwnedCommand;
    checks["composite-ready-plan-is-one-command"] =
        PrepareRetainedReplay(replay).admitted()
        && PrepareRetainedReplay(replay).mayOpenOneOwnedCommand;
    const auto& real=RealKernelChecks();checks.insert(real.begin(),real.end());return checks;
}
} // namespace core3d::spatial_sweep::debug

inline std::map<std::string, bool> Core3DDebugSpatialSweepK0K1Probe() {
    return core3d::spatial_sweep::debug::Probe();
}


inline std::map<std::string, bool> Core3DDebugSpatialSweepK2Probe() {
    return core3d::spatial_sweep::debug::K2Probe();
}
