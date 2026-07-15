#include "TranslationInvarianceTests.h"

#include "core/geometry/GeometryCompiler.h"
#include "core/machine/RotaryTrajectoryBuilder.h"
#include "core/machining/TubeCutBoundary.h"
#include "core/machining/TubeSection.h"
#include "core/nc/NcProgramBuilder.h"
#include "core/planning/ProcessPlanBuilder.h"
#include "core/topology/PathTopology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <type_traits>

namespace
{
    using namespace cadcam;
    using geometry::EntityId;
    using geometry::Path3D;
    using geometry::SourceEntity;
    using geometry::SourceGeometryKind;
    using geometry::Vector3d;

    int failures = 0;

    void check(bool condition, const char* message)
    {
        if (!condition)
        {
            ++failures;
            std::cerr << "FAILED: " << message << '\n';
        }
    }

    OperationContext context(const QString& name)
    {
        return createOperationContext(name);
    }

    TaskContext task(const QString& name)
    {
        TaskContext value;
        value.operationContext = context(name);
        return value;
    }

    Vector3d translated(const Vector3d& point, const Vector3d& offset)
    {
        return { point.x + offset.x, point.y + offset.y, point.z + offset.z };
    }

    double coordinateLimit(const Vector3d& point)
    {
        const double magnitude = std::max({ std::abs(point.x), std::abs(point.y),
            std::abs(point.z), 1.0 });
        const double next = std::nextafter(magnitude, std::numeric_limits<double>::infinity());
        return std::max(1.0e-9, (next - magnitude) * 8.0);
    }

    bool sameTranslatedPath(const Path3D& base, const Path3D& moved, const Vector3d& offset)
    {
        if (base.sourceEntityId != moved.sourceEntityId || base.sourceKind != moved.sourceKind
            || base.closed != moved.closed || base.vertices.size() != moved.vertices.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < base.vertices.size(); ++index)
        {
            if (base.vertices[index].sourceParameter != moved.vertices[index].sourceParameter)
            {
                return false;
            }
            const Vector3d restored
            {
                moved.vertices[index].position.x - offset.x,
                moved.vertices[index].position.y - offset.y,
                moved.vertices[index].position.z - offset.z
            };
            const double limit = coordinateLimit(moved.vertices[index].position);
            if (std::abs(restored.x - base.vertices[index].position.x) > limit
                || std::abs(restored.y - base.vertices[index].position.y) > limit
                || std::abs(restored.z - base.vertices[index].position.z) > limit)
            {
                return false;
            }
        }
        return true;
    }

    bool sameSeams
    (
        const std::array<machining::SeamWindingResult, 4>& left,
        const std::array<machining::SeamWindingResult, 4>& right
    )
    {
        for (std::size_t index = 0; index < left.size(); ++index)
        {
            if (left[index].usable != right[index].usable
                || left[index].winding != right[index].winding
                || left[index].positiveCrossingCount != right[index].positiveCrossingCount
                || left[index].negativeCrossingCount != right[index].negativeCrossingCount
                || left[index].touchCount != right[index].touchCount
                || left[index].overlapRunCount != right[index].overlapRunCount)
            {
                return false;
            }
        }
        return true;
    }

    SourceEntity translateSource(const SourceEntity& source, const Vector3d& offset)
    {
        SourceEntity result = source;
        std::visit([&offset](auto& geometry)
        {
            using Geometry = std::decay_t<decltype(geometry)>;
            if constexpr (std::is_same_v<Geometry, geometry::LineGeometry>)
            {
                geometry.start = translated(geometry.start, offset);
                geometry.end = translated(geometry.end, offset);
            }
            else if constexpr (std::is_same_v<Geometry, geometry::CircleGeometry>
                || std::is_same_v<Geometry, geometry::ArcGeometry>
                || std::is_same_v<Geometry, geometry::EllipseGeometry>)
            {
                geometry.center = translated(geometry.center, offset);
            }
            else if constexpr (std::is_same_v<Geometry, geometry::PolylineGeometry>)
            {
                for (auto& primitive : geometry.segments)
                {
                    std::visit([&offset](auto& segment)
                    {
                        using Segment = std::decay_t<decltype(segment)>;
                        if constexpr (std::is_same_v<Segment, geometry::LineGeometry>)
                        {
                            segment.start = translated(segment.start, offset);
                            segment.end = translated(segment.end, offset);
                        }
                        else
                        {
                            segment.center = translated(segment.center, offset);
                        }
                    }, primitive);
                }
            }
            else if constexpr (std::is_same_v<Geometry, geometry::SplineGeometry>)
            {
                for (Vector3d& point : geometry.controlPoints) point = translated(point, offset);
                for (Vector3d& point : geometry.fitPoints) point = translated(point, offset);
            }
        }, result.geometry);
        return result;
    }

    std::vector<SourceEntity> geometryFixtures()
    {
        std::vector<SourceEntity> fixtures;
        fixtures.push_back({ 1, SourceGeometryKind::Line,
            geometry::LineGeometry{ { 1, 2, 3 }, { 2, 2.5, 3.25 } } });
        fixtures.push_back({ 2, SourceGeometryKind::Arc,
            geometry::ArcGeometry{ { 5, 6, 7 }, { 1, 0, 0 }, { 0, 1, 0 },
                4, 0.2, 2.4 } });
        fixtures.push_back({ 3, SourceGeometryKind::Circle,
            geometry::CircleGeometry{ { -2, 3, 4 }, { 1, 0, 0 }, { 0, 0, 1 }, 3 } });
        fixtures.push_back({ 4, SourceGeometryKind::Ellipse,
            geometry::EllipseGeometry{ { 2, -4, 3 }, { 6, 0, 0 }, { 0, 0, 2 },
                0, 6.28318530717958647692, true } });

        geometry::PolylineGeometry polyline;
        polyline.sourceVertexCount = 3;
        polyline.closed = false;
        polyline.segments.emplace_back(geometry::LineGeometry{ { 0, 0, 0 }, { 2, 0, 0 } });
        polyline.segments.emplace_back(geometry::ArcGeometry
            { { 2, 1, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, 1,
              -1.57079632679489661923, 0.0 });
        fixtures.push_back({ 5, SourceGeometryKind::Polyline, polyline });

        geometry::SplineGeometry spline;
        spline.degree = 3;
        spline.controlPoints = { { 0, 0, 0 }, { 2, 4, 1 }, { 6, 4, -1 }, { 8, 0, 0 } };
        spline.weights = { 1, 0.75, 1.25, 1 };
        spline.knots = { 0, 0, 0, 0, 1, 1, 1, 1 };
        spline.rational = true;
        spline.parameterStart = 0;
        spline.parameterEnd = 1;
        fixtures.push_back({ 6, SourceGeometryKind::Spline, spline });
        return fixtures;
    }

    topology::TopologyInput rectangleInput(const Vector3d& offset, double closingGap = 0.0)
    {
        topology::TopologyInput input;
        input.contentRevision = 1;
        const std::array<Vector3d, 4> points
        {{
            translated({ 10, -50, -30 }, offset),
            translated({ 10, 50, -30 }, offset),
            translated({ 10, 50, 30 }, offset),
            translated({ 10, -50, 30 }, offset)
        }};
        for (std::size_t index = 0; index < 4; ++index)
        {
            topology::TopologyPathRecord record;
            record.sourceIndex = index;
            record.entityId = 100 + index;
            record.sourceKind = SourceGeometryKind::Line;
            record.points = { points[index], points[(index + 1) % 4] };
            if (index == 3) record.points.back().y += closingGap;
            input.records.push_back(record);
        }
        topology::TopologyPathRecord internal;
        internal.sourceIndex = 4;
        internal.entityId = 104;
        internal.sourceKind = SourceGeometryKind::Line;
        internal.points =
            { translated({ 10, -10, 0 }, offset), translated({ 10, 10, 0 }, offset) };
        input.records.push_back(std::move(internal));
        return input;
    }

    OperationResult<topology::PathTopology> buildTopology(const topology::TopologyInput& input)
    {
        return topology::PathTopologyBuilder{}.build
            (input, topology::PathTopologyTolerance{}, task(QStringLiteral("translation-topology")));
    }

    void testGeometryTranslation(const std::array<Vector3d, 4>& offsets)
    {
        geometry::GeometryCompiler compiler;
        geometry::SamplingPolicy policy;
        const geometry::PathCompileOptions options;
        for (const SourceEntity& fixture : geometryFixtures())
        {
            const auto base = compiler.compile
                (fixture, policy, options, context(QStringLiteral("translation-geometry-base")));
            check(base.succeeded() && base.value.has_value(), "translation geometry base compiles");
            if (!base.value.has_value()) continue;
            for (const Vector3d& offset : offsets)
            {
                const auto moved = compiler.compile(translateSource(fixture, offset), policy, options,
                    context(QStringLiteral("translation-geometry-moved")));
                check(moved.succeeded() && moved.value.has_value()
                    && sameTranslatedPath(*base.value, *moved.value, offset),
                    "GeometryCompiler is translation invariant");
            }
        }
    }

    void testTopologyAndMachiningTranslation(const std::array<Vector3d, 4>& offsets)
    {
        std::vector<EntityId> ids{ 100, 101, 102, 103 };
        const auto baseInput = rectangleInput(offsets[0]);
        const auto baseTopology = buildTopology(baseInput);
        check(baseTopology.succeeded() && baseTopology.value.has_value(),
            "translation topology base builds");
        if (!baseTopology.value.has_value()) return;
        const auto baseLoop = baseTopology.value->extractBestLoop(ids);
        const auto baseSection = machining::TubeSectionAnalyzer::buildFromSelection
            (baseInput, *baseTopology.value, ids, machining::TubeSectionPolicy{},
             context(QStringLiteral("translation-section-base")));
        check(baseLoop.succeeded() && baseLoop.value.has_value()
            && baseSection.succeeded() && baseSection.value.has_value(),
            "translation topology loop and section base solve");
        if (!baseLoop.value.has_value() || !baseSection.value.has_value()) return;
        const auto baseCut = machining::TubeCutBoundaryClassifier::analyze
            (baseLoop.value->orderedPath, ids, baseLoop.value->maximumJoinGap,
             baseSection.value->geometry, context(QStringLiteral("translation-cut-base")));
        check(baseCut.succeeded() && baseCut.value.has_value(),
            "translation cut boundary base solves");
        const auto baseInternal = machining::TubeSectionAnalyzer::classifyInternalPaths
            (baseInput, *baseTopology.value, *baseSection.value,
             machining::TubeSectionPolicy{}, context(QStringLiteral("translation-internal-base")));
        check(baseInternal.succeeded() && baseInternal.value.has_value()
            && baseInternal.value->physicalInteriorEntityIds == std::vector<EntityId>{ 104 },
            "translation internal path base classifies");

        for (const Vector3d& offset : offsets)
        {
            const auto input = rectangleInput(offset);
            const auto topology = buildTopology(input);
            check(topology.succeeded() && topology.value.has_value(),
                "translated topology builds");
            if (!topology.value.has_value()) continue;
            const auto loop = topology.value->extractBestLoop(ids);
            const auto section = machining::TubeSectionAnalyzer::buildFromSelection
                (input, *topology.value, ids, machining::TubeSectionPolicy{},
                 context(QStringLiteral("translation-section")));
            check(loop.succeeded() && loop.value.has_value()
                && loop.value->usedEntityIds == baseLoop.value->usedEntityIds
                && section.succeeded() && section.value.has_value(),
                "topology and section identities are translation invariant");
            if (!loop.value.has_value() || !section.value.has_value()) continue;
            const double limit = coordinateLimit({ section.value->centerX,
                section.value->geometry.centerY, section.value->geometry.centerZ });
            check(std::abs(section.value->geometry.yLength - baseSection.value->geometry.yLength) <= limit
                && std::abs(section.value->geometry.zWidth - baseSection.value->geometry.zWidth) <= limit
                && std::abs(section.value->geometry.perimeter - baseSection.value->geometry.perimeter) <= limit
                && std::abs(section.value->geometry.centerY - offset.y
                    - baseSection.value->geometry.centerY) <= limit
                && std::abs(section.value->geometry.centerZ - offset.z
                    - baseSection.value->geometry.centerZ) <= limit,
                "TubeSection dimensions and center are translation invariant");
            const auto cut = machining::TubeCutBoundaryClassifier::analyze
                (loop.value->orderedPath, ids, loop.value->maximumJoinGap,
                 section.value->geometry, context(QStringLiteral("translation-cut")));
            check(cut.succeeded() && cut.value.has_value()
                && baseCut.value.has_value()
                && cut.value->result == baseCut.value->result
                && cut.value->winding == baseCut.value->winding
                && sameSeams(cut.value->seamResults, baseCut.value->seamResults),
                "TubeCutBoundary winding and seams are translation invariant");
            const auto internal = machining::TubeSectionAnalyzer::classifyInternalPaths
                (input, *topology.value, *section.value, machining::TubeSectionPolicy{},
                 context(QStringLiteral("translation-internal")));
            check(internal.succeeded() && internal.value.has_value()
                && baseInternal.value.has_value()
                && internal.value->physicalInteriorEntityIds
                    == baseInternal.value->physicalInteriorEntityIds,
                "internal path EntityIds are translation invariant");
        }

        const Vector3d far = offsets.back();
        const auto acceptedTopology = buildTopology(rectangleInput(far, 5.0e-6));
        const auto rejectedTopology = buildTopology(rectangleInput(far, 2.0e-5));
        const auto accepted = acceptedTopology.value->extractBestLoop(ids);
        const auto rejected = rejectedTopology.value->extractBestLoop(ids);
        check(accepted.succeeded() && accepted.value->connectedLoop,
            "far 5e-6 numerical join remains closed");
        check(!rejected.succeeded(), "far 2e-5 physical gap remains open");
    }

    Path3D makePath(EntityId id, const std::vector<Vector3d>& points)
    {
        Path3D path;
        path.sourceEntityId = id;
        path.sourceKind = SourceGeometryKind::Line;
        for (std::size_t index = 0; index < points.size(); ++index)
            path.vertices.push_back({ points[index], static_cast<double>(index) });
        return path;
    }

    void testPlanTrajectoryAndNcTranslation(const std::array<Vector3d, 4>& offsets)
    {
        std::vector<EntityId> baseOrder;
        std::vector<machine::EntityTrajectory> baseTrajectories;
        std::optional<nc::NcProgram> baseProgram;
        for (std::size_t translationIndex = 0; translationIndex < offsets.size(); ++translationIndex)
        {
            const Vector3d offset = offsets[translationIndex];
            planning::ProcessPlanningInput planningInput;
            planningInput.contentRevision = 1;
            planningInput.processStateRevision = 1;
            const Path3D first = makePath(201,
                { translated({ 0, 0, 10 }, offset), translated({ 10, 0, 10 }, offset) });
            const Path3D second = makePath(202,
                { translated({ 20, 0, 10 }, offset), translated({ 30, 0, 10 }, offset) });
            planningInput.entities =
            {
                { 201, 0, SourceGeometryKind::Line, first },
                { 202, 1, SourceGeometryKind::Line, second }
            };
            planningInput.topologyInput.contentRevision = 1;
            planningInput.topologyInput.records =
            {
                { 0, 201, SourceGeometryKind::Line,
                    { first.vertices[0].position, first.vertices[1].position }, false },
                { 1, 202, SourceGeometryKind::Line,
                    { second.vertices[0].position, second.vertices[1].position }, false }
            };
            const auto topology = buildTopology(planningInput.topologyInput);
            planningInput.topology = &*topology.value;
            planning::ProcessPlanningPolicy planningPolicy;
            planningPolicy.initialPosition = translated({ 0, 0, 500 }, offset);
            const auto plan = planning::ProcessPlanBuilder::build
                (planningInput, planningPolicy, context(QStringLiteral("translation-plan")));
            check(plan.succeeded() && plan.value.has_value(),
                "translated ProcessPlan builds");
            if (!plan.value.has_value()) continue;
            std::vector<EntityId> order;
            for (const auto& assignment : plan.value->assignments) order.push_back(assignment.entityId);
            if (translationIndex == 0) baseOrder = order;
            else check(order == baseOrder, "ProcessPlan order is translation invariant");

            machine::RotaryTrajectoryInput trajectoryInput;
            trajectoryInput.contentRevision = 1;
            trajectoryInput.processStateRevision = 1;
            for (const auto& assignment : plan.value->assignments)
            {
                const Path3D& path = assignment.entityId == 201 ? first : second;
                trajectoryInput.entities.push_back
                    ({ assignment.entityId, assignment.entityId == 201 ? 0U : 1U,
                       SourceGeometryKind::Line, assignment.processOrder,
                       assignment.continuousGroupId, false, true, true, path });
            }
            trajectoryInput.processGroups = plan.value->groups;
            machine::RotaryMachinePolicy machinePolicy;
            machinePolicy.rotaryAxisY = offset.y;
            machinePolicy.rotaryAxisZ = offset.z;
            machinePolicy.tubeCenterY = offset.y;
            machinePolicy.tubeCenterZ = offset.z;
            machinePolicy.useInitialMachinePoint = true;
            machinePolicy.initialMachinePoint =
                { offset.x, offset.y, offset.z + 500, 0 };
            const auto trajectory = machine::RotaryTrajectoryBuilder::build
                (trajectoryInput, machinePolicy, task(QStringLiteral("translation-trajectory")));
            check(trajectory.succeeded() && trajectory.value.has_value(),
                "translated MachineTrajectory builds");
            if (!trajectory.value.has_value()) continue;
            if (translationIndex == 0)
            {
                baseTrajectories = trajectory.value->entities;
            }
            else
            {
                check(trajectory.value->entities.size() == baseTrajectories.size(),
                    "MachineTrajectory entity count is translation invariant");
                for (std::size_t entity = 0; entity < trajectory.value->entities.size(); ++entity)
                {
                    const auto& moved = trajectory.value->entities[entity];
                    const auto& base = baseTrajectories[entity];
                    check(moved.entityId == base.entityId
                        && moved.approachMoves.size() == base.approachMoves.size()
                        && moved.cuttingMoves.size() == base.cuttingMoves.size()
                        && moved.overcutMoves.size() == base.overcutMoves.size(),
                        "MachineTrajectory move structure is translation invariant");
                    const auto compareMoves = [&offset]
                    (
                        const std::vector<machine::MachineMove>& baseMoves,
                        const std::vector<machine::MachineMove>& movedMoves
                    )
                    {
                        if (baseMoves.size() != movedMoves.size()) return false;
                        for (std::size_t move = 0; move < baseMoves.size(); ++move)
                        {
                            const double limit = coordinateLimit
                                ({ movedMoves[move].target.x, movedMoves[move].target.y,
                                    movedMoves[move].target.z });
                            if (baseMoves[move].kind != movedMoves[move].kind
                                || baseMoves[move].entityId != movedMoves[move].entityId
                                || baseMoves[move].processGroupId
                                    != movedMoves[move].processGroupId
                                || std::abs(baseMoves[move].target.aDegrees
                                    - movedMoves[move].target.aDegrees) > 1.0e-9
                                || std::abs(movedMoves[move].target.x - offset.x
                                    - baseMoves[move].target.x) > limit
                                || std::abs(movedMoves[move].target.y - offset.y
                                    - baseMoves[move].target.y) > limit
                                || std::abs(movedMoves[move].target.z - offset.z
                                    - baseMoves[move].target.z) > limit)
                            {
                                return false;
                            }
                        }
                        return true;
                    };
                    check(compareMoves(base.approachMoves, moved.approachMoves)
                        && compareMoves(base.cuttingMoves, moved.cuttingMoves)
                        && compareMoves(base.overcutMoves, moved.overcutMoves),
                        "MachineTrajectory poses and A axis are translation invariant");
                }
            }

            std::vector<nc::NcEntityMetadata> metadata;
            for (const machine::EntityTrajectory& entity : trajectory.value->entities)
            {
                nc::NcEntityMetadata entry;
                entry.entityId = entity.entityId;
                entry.sourceKind = entity.sourceKind;
                entry.sourceIndex = entity.sourceIndex;
                entry.processOrder = entity.processOrder;
                entry.processGroupId = entity.processGroupId;
                metadata.push_back(std::move(entry));
            }
            const auto program = nc::NcProgramBuilder::buildRotary
                (*trajectory.value, metadata, context(QStringLiteral("translation-nc")));
            check(program.succeeded() && program.value.has_value(),
                "translated NcProgram builds");
            if (program.value.has_value())
            {
                if (!baseProgram.has_value()) baseProgram = *program.value;
                else
                {
                    check(program.value->mode == baseProgram->mode
                        && program.value->entities.size() == baseProgram->entities.size(),
                        "NcProgram block structure is translation invariant");
                    for (std::size_t entity = 0;
                        entity < program.value->entities.size()
                            && entity < baseProgram->entities.size(); ++entity)
                    {
                        check(program.value->entities[entity].metadata.entityId
                                == baseProgram->entities[entity].metadata.entityId
                            && program.value->entities[entity].motions.size()
                                == baseProgram->entities[entity].motions.size(),
                            "NcProgram entity order and motion count are translation invariant");
                    }
                }
            }
        }
    }
}

int runTranslationInvarianceTests()
{
    const std::array<Vector3d, 4> offsets
    {{
        { 0, 0, 0 },
        { 100000, -200000, 300000 },
        { 10000000, -20000000, 30000000 },
        { 1000000000, -2000000000, 3000000000 }
    }};
    testGeometryTranslation(offsets);
    testTopologyAndMachiningTranslation(offsets);
    testPlanTrajectoryAndNcTranslation(offsets);
    return failures;
}
