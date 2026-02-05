/**
 * @file phase1.cpp
 * @brief Phase 1 integration implementation
 */

#include <openpanelcam/phase1/phase1.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>
#include <openpanelcam/io/step_reader.h>
#include <openpanelcam/phase1/geometry_healer.h>
#include <openpanelcam/phase1/fag_builder.h>
#include <openpanelcam/phase1/base_face_identifier.h>
#include <openpanelcam/phase1/bend_classifier.h>

#include <chrono>

namespace openpanelcam {

Phase1Output parseSTEPFile(const std::string& filePath, const Phase1Config& config) {
    Phase1Output output;
    output.filePath = filePath;

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // =====================================================================
        // Step 1: Load STEP file
        // =====================================================================
        LOG_INFO("Phase 1: Loading STEP file: {}", filePath);

        STEPReader reader;
        TopoDS_Shape shape = reader.read(filePath);

        if (shape.IsNull()) {
            output.success = false;
            output.errorMessage = "Failed to load STEP file (null shape)";
            LOG_ERROR(output.errorMessage);
            return output;
        }

        output.originalShape = shape;
        LOG_INFO("STEP file loaded successfully");

        // =====================================================================
        // Step 2: Heal geometry (optional)
        // =====================================================================
        TopoDS_Shape workingShape = shape;

        if (config.healGeometry) {
            LOG_INFO("Phase 1: Healing geometry");

            GeometryHealer healer;
            healer.setTolerance(config.healingTolerance);

            GeometryHealer::HealingOptions options;
            options.fixSmallEdges = config.fixSmallEdges;
            options.fixSmallFaces = config.fixSmallFaces;
            options.sewFaces = config.sewFaces;
            options.fixOrientation = config.fixOrientation;

            GeometryHealer::HealingResult result = healer.heal(shape, options);

            if (result.success) {
                workingShape = result.healedShape;
                output.healedShape = result.healedShape;
                output.wasHealed = true;
                output.healingIssuesFixed = result.issuesFixed;

                LOG_INFO("Geometry healed: {} issues fixed", result.issuesFixed);

                // Log warnings
                for (const auto& warning : result.warnings) {
                    output.warnings.push_back(warning);
                    if (config.verbose) {
                        LOG_WARNING("Healing: {}", warning);
                    }
                }
            } else {
                output.warnings.push_back("Geometry healing failed: " + result.errorMessage);
                LOG_WARNING("Geometry healing failed, using original shape");
                // Continue with original shape
            }
        } else {
            LOG_INFO("Geometry healing disabled");
        }

        // =====================================================================
        // Step 3: Build Face-Adjacency Graph
        // =====================================================================
        LOG_INFO("Phase 1: Building Face-Adjacency Graph");

        FAGBuilder fagBuilder;
        fagBuilder.setMinBendRadius(config.minBendRadius);
        fagBuilder.setMinFaceArea(config.minFaceArea);
        fagBuilder.enableSdfValidation(config.sdfValidation);

        FaceAdjacencyGraph fag = fagBuilder.build(workingShape);

        if (fag.nodeCount() == 0) {
            output.success = false;
            output.errorMessage = "FAG construction failed (no nodes)";
            LOG_ERROR(output.errorMessage);
            return output;
        }

        output.fag = fag;
        output.totalFaces = fag.nodeCount();
        output.bendCount = fag.bendCount();

        LOG_INFO("FAG built: {} faces, {} bend edges", fag.nodeCount(), fag.bendCount());

        // Count face types
        for (const auto& node : fag.nodes()) {
            if (node.type == FaceType::PLANAR) {
                output.planarFaces++;
            } else if (node.type == FaceType::CYLINDRICAL) {
                output.cylindricalFaces++;
            } else {
                output.otherFaces++;
            }
        }

        if (config.verbose) {
            LOG_DEBUG("Face types: {} planar, {} cylindrical, {} other",
                     output.planarFaces, output.cylindricalFaces, output.otherFaces);
        }

        // =====================================================================
        // Step 4: Identify base face
        // =====================================================================
        LOG_INFO("Phase 1: Identifying base face");

        BaseFaceIdentifier baseFaceIdentifier;
        baseFaceIdentifier.setWeights(
            config.weightArea,
            config.weightConnectivity,
            config.weightCentrality,
            config.weightOrientation
        );
        baseFaceIdentifier.setPreferredNormal(config.preferredNormal);

        int baseFaceId = baseFaceIdentifier.identify(fag);

        if (baseFaceId < 0) {
            output.success = false;
            output.errorMessage = "Base face identification failed";
            LOG_ERROR(output.errorMessage);
            return output;
        }

        output.baseFaceId = baseFaceId;

        // Get base face score
        auto candidates = baseFaceIdentifier.getCandidates();
        if (!candidates.empty()) {
            output.baseFaceScore = candidates[0].score;
        }

        LOG_INFO("Base face identified: node {} (score: {:.3f})",
                 baseFaceId, output.baseFaceScore);

        // Set base face in FAG and compute flange levels
        fag.setBaseFace(baseFaceId);

        // =====================================================================
        // Step 5: Classify bends
        // =====================================================================
        LOG_INFO("Phase 1: Classifying bends");

        BendClassifier bendClassifier;
        bendClassifier.setThickness(config.thickness);
        bendClassifier.setHemAngleThreshold(config.hemAngleThreshold);
        bendClassifier.setKFactor(config.kFactor);

        if (!workingShape.IsNull() && workingShape.ShapeType() == TopAbs_SOLID) {
            bendClassifier.setSolid(TopoDS::Solid(workingShape));
        }

        std::vector<BendFeature> bends = bendClassifier.classify(fag, baseFaceId);

        output.bends = bends;
        output.bendCount = static_cast<int>(bends.size());

        // Count bend types
        for (const auto& bend : bends) {
            if (bend.direction == BendDirection::HEM) {
                output.hemCount++;
            } else if (bend.direction == BendDirection::BEND_UP) {
                output.upBendCount++;
            } else if (bend.direction == BendDirection::BEND_DOWN) {
                output.downBendCount++;
            }
        }

        LOG_INFO("Classified {} bends: {} UP, {} DOWN, {} HEM",
                 output.bendCount, output.upBendCount, output.downBendCount, output.hemCount);

        // =====================================================================
        // Success
        // =====================================================================
        output.success = true;

        auto endTime = std::chrono::high_resolution_clock::now();
        output.parseTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        LOG_INFO("Phase 1 completed successfully in {:.2f} ms", output.parseTime);

        if (config.verbose) {
            printPhase1Summary(output);
        }

    } catch (const Phase1Exception& e) {
        output.success = false;
        output.errorMessage = std::string("Phase1Exception: ") + e.what();
        LOG_ERROR("Phase 1 failed: {}", output.errorMessage);
    } catch (const std::exception& e) {
        output.success = false;
        output.errorMessage = std::string("Exception: ") + e.what();
        LOG_ERROR("Phase 1 failed: {}", output.errorMessage);
    }

    return output;
}

bool validatePhase1Output(const Phase1Output& output) {
    if (!output.success) {
        LOG_ERROR("Validation failed: output.success = false");
        return false;
    }

    // Check base face
    if (output.baseFaceId < 0) {
        LOG_ERROR("Validation failed: invalid base face ID");
        return false;
    }

    if (output.baseFaceId >= output.fag.nodeCount()) {
        LOG_ERROR("Validation failed: base face ID out of range");
        return false;
    }

    // Check FAG is valid
    if (output.fag.nodeCount() == 0) {
        LOG_ERROR("Validation failed: FAG has no nodes");
        return false;
    }

    // Check bends
    for (const auto& bend : output.bends) {
        // Check base face reference
        if (bend.baseFaceId < 0 || bend.baseFaceId >= output.fag.nodeCount()) {
            LOG_ERROR("Validation failed: bend {} has invalid base face ID", bend.id);
            return false;
        }

        // Check flange face reference
        if (bend.flangeFaceId < 0 || bend.flangeFaceId >= output.fag.nodeCount()) {
            LOG_ERROR("Validation failed: bend {} has invalid flange face ID", bend.id);
            return false;
        }

        // Check bend face is not null
        if (bend.bendFace.IsNull()) {
            LOG_ERROR("Validation failed: bend {} has null bend face", bend.id);
            return false;
        }

        // Check angle is reasonable
        if (bend.targetAngle < 0.0 || bend.targetAngle > 180.0) {
            LOG_ERROR("Validation failed: bend {} has invalid angle {:.1f}",
                     bend.id, bend.targetAngle);
            return false;
        }

        // Check radius is positive
        if (bend.internalRadius < 0.0) {
            LOG_ERROR("Validation failed: bend {} has invalid radius {:.2f}",
                     bend.id, bend.internalRadius);
            return false;
        }
    }

    // Check for circular dependencies in bend sequence
    // (This is a simple check - full topological sort would be better)
    for (const auto& bend : output.bends) {
        for (int beforeId : bend.mustBendBefore) {
            if (beforeId == bend.id) {
                LOG_ERROR("Validation failed: bend {} has circular dependency (self)", bend.id);
                return false;
            }
        }
    }

    LOG_INFO("Validation passed");
    return true;
}

void printPhase1Summary(const Phase1Output& output) {
    LOG_INFO("=== Phase 1 Summary ===");
    LOG_INFO("File: {}", output.filePath);
    LOG_INFO("Parse time: {:.2f} ms", output.parseTime);
    LOG_INFO("Success: {}", output.success ? "YES" : "NO");

    if (!output.success) {
        LOG_INFO("Error: {}", output.errorMessage);
        return;
    }

    // Geometry
    LOG_INFO("--- Geometry ---");
    LOG_INFO("Healing: {}", output.wasHealed ? "YES" : "NO");
    if (output.wasHealed) {
        LOG_INFO("  Issues fixed: {}", output.healingIssuesFixed);
    }

    // FAG
    LOG_INFO("--- Face-Adjacency Graph ---");
    LOG_INFO("Total faces: {}", output.totalFaces);
    LOG_INFO("  Planar: {}", output.planarFaces);
    LOG_INFO("  Cylindrical: {}", output.cylindricalFaces);
    LOG_INFO("  Other: {}", output.otherFaces);
    LOG_INFO("Bend edges: {}", output.fag.bendCount());

    // Base face
    LOG_INFO("--- Base Face ---");
    LOG_INFO("Base face ID: {}", output.baseFaceId);
    LOG_INFO("Base face score: {:.3f}", output.baseFaceScore);

    // Bends
    LOG_INFO("--- Bends ---");
    LOG_INFO("Total bends: {}", output.bendCount);
    LOG_INFO("  UP bends: {}", output.upBendCount);
    LOG_INFO("  DOWN bends: {}", output.downBendCount);
    LOG_INFO("  HEMs: {}", output.hemCount);

    // List all bends
    for (const auto& bend : output.bends) {
        std::string dirStr;
        if (bend.direction == BendDirection::BEND_UP) {
            dirStr = "UP";
        } else if (bend.direction == BendDirection::BEND_DOWN) {
            dirStr = "DOWN";
        } else if (bend.direction == BendDirection::HEM) {
            dirStr = "HEM";
        } else {
            dirStr = "UNKNOWN";
        }

        LOG_INFO("  Bend {}: {} {:.1f}° (R={:.2f} mm, L={:.2f} mm)",
                 bend.id,
                 dirStr,
                 bend.targetAngle,
                 bend.internalRadius,
                 bend.bendLineLength);
    }

    // Warnings
    if (!output.warnings.empty()) {
        LOG_INFO("--- Warnings ---");
        for (const auto& warning : output.warnings) {
            LOG_WARNING("  {}", warning);
        }
    }

    LOG_INFO("======================");
}

} // namespace openpanelcam
