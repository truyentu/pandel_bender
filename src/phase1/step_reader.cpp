/**
 * @file step_reader.cpp
 * @brief STEP file reader implementation
 */

#include <openpanelcam/phase1/step_reader.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>
#include <openpanelcam/core/constants.h>

// OCCT STEP I/O
#include <STEPControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Transfer_TransientProcess.hxx>

// Topology
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Builder.hxx>

// Validation
#include <BRepCheck_Analyzer.hxx>

// Unit conversion
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Trsf.hxx>

#include <filesystem>

namespace openpanelcam {

STEPReader::STEPReader()
    : m_unitConverted(false)
{
}

STEPReader::~STEPReader() {
    // OCCT handles will be released automatically
}

bool STEPReader::load(const std::string& filepath) {
    m_filepath = filepath;
    m_lastError.clear();

    LOG_INFO("Loading STEP file: {}", filepath);

    // Check file exists
    if (!std::filesystem::exists(filepath)) {
        m_lastError = "File not found: " + filepath;
        LOG_ERROR("{}", m_lastError);
        THROW_PHASE1(ErrorCode::STEP_LOAD_FAILED,
                    "STEP file not found",
                    m_lastError);
        return false;
    }

    try {
        // Create STEP reader
        STEPControl_Reader reader;

        // Read file
        IFSelect_ReturnStatus status = reader.ReadFile(filepath.c_str());

        if (status != IFSelect_RetDone) {
            m_lastError = "Failed to read STEP file";
            LOG_ERROR("{}", m_lastError);
            THROW_PHASE1(ErrorCode::STEP_LOAD_FAILED,
                        "STEP read failed",
                        "Status: " + std::to_string(static_cast<int>(status)));
            return false;
        }

        LOG_DEBUG("STEP file read successfully");

        // Transfer roots to shape
        int nbRoots = reader.NbRootsForTransfer();
        LOG_DEBUG("Found {} root(s) for transfer", nbRoots);

        if (nbRoots == 0) {
            m_lastError = "No roots found in STEP file";
            LOG_ERROR("{}", m_lastError);
            THROW_PHASE1(ErrorCode::NO_SOLID_FOUND,
                        "No geometry in STEP file",
                        "File contains no transferable roots");
            return false;
        }

        // Transfer all roots
        reader.TransferRoots();

        // Get number of shapes
        int nbShapes = reader.NbShapes();
        LOG_DEBUG("Transferred {} shape(s)", nbShapes);

        if (nbShapes == 0) {
            m_lastError = "No shapes transferred from STEP file";
            LOG_ERROR("{}", m_lastError);
            THROW_PHASE1(ErrorCode::NO_SOLID_FOUND,
                        "Shape transfer failed",
                        "No shapes could be transferred");
            return false;
        }

        // Get the shape (if multiple, create compound)
        if (nbShapes == 1) {
            m_shape = reader.Shape(1);
        } else {
            // Multiple shapes - create compound
            LOG_WARNING("Multiple shapes found ({}), creating compound", nbShapes);
            BRep_Builder builder;
            TopoDS_Compound compound;
            builder.MakeCompound(compound);

            for (int i = 1; i <= nbShapes; i++) {
                builder.Add(compound, reader.Shape(i));
            }

            m_shape = compound;
        }

        // Validate shape
        if (!validateShape()) {
            return false;
        }

        // Extract metadata
        extractMetadata();

        // Convert units if needed
        convertUnitsToMM();

        LOG_INFO("STEP file loaded successfully: {} solids, {} units{}",
                 getSolidCount(),
                 m_units,
                 m_unitConverted ? " (converted to MM)" : "");

        return true;

    } catch (const Standard_Failure& e) {
        m_lastError = "OCCT exception: " + std::string(e.GetMessageString());
        LOG_ERROR("{}", m_lastError);
        THROW_OCCT("STEP loading failed", m_lastError);
        return false;
    } catch (const std::exception& e) {
        m_lastError = "Exception: " + std::string(e.what());
        LOG_ERROR("{}", m_lastError);
        return false;
    }
}

bool STEPReader::validateShape() {
    if (m_shape.IsNull()) {
        m_lastError = "Shape is null";
        LOG_ERROR("{}", m_lastError);
        return false;
    }

    // Check if shape contains at least one solid
    int solidCount = countSolids();
    if (solidCount == 0) {
        m_lastError = "No solids found in shape";
        LOG_WARNING("{}", m_lastError);
        // Don't fail - might be a surface or compound
    }

    // Run OCCT validation
    BRepCheck_Analyzer analyzer(m_shape);
    if (!analyzer.IsValid()) {
        LOG_WARNING("Shape has validation issues (will attempt healing later)");
        // Don't fail - GeometryHealer will fix issues
    }

    return true;
}

void STEPReader::extractMetadata() {
    // Extract units
    // OCCT stores units in Interface_Static
    const char* unitsStr = Interface_Static::CVal("xstep.cascade.unit");
    if (unitsStr && strlen(unitsStr) > 0) {
        m_originalUnits = unitsStr;

        // Normalize unit string
        std::string unitLower = m_originalUnits;
        std::transform(unitLower.begin(), unitLower.end(), unitLower.begin(), ::tolower);

        if (unitLower.find("mm") != std::string::npos ||
            unitLower.find("millimeter") != std::string::npos) {
            m_units = "MM";
        } else if (unitLower.find("inch") != std::string::npos ||
                   unitLower.find("in") != std::string::npos) {
            m_units = "INCH";
        } else if (unitLower.find("m") != std::string::npos ||
                   unitLower.find("meter") != std::string::npos) {
            m_units = "METER";
        } else {
            m_units = "UNKNOWN";
        }
    } else {
        // Default assumption
        m_units = "MM";
        m_originalUnits = "MM (assumed)";
    }

    // Extract part name (simplified - full implementation needs STEP entity parsing)
    // For now, use filename
    std::filesystem::path path(m_filepath);
    m_partName = path.stem().string();

    // Detect AP version (simplified)
    // Full implementation would parse STEP header
    m_apVersion = "UNKNOWN";
}

void STEPReader::convertUnitsToMM() {
    m_unitConverted = false;

    if (m_units == "MM") {
        // Already in MM
        return;
    }

    double scaleFactor = 1.0;

    if (m_units == "INCH") {
        scaleFactor = 25.4;  // 1 inch = 25.4 mm
    } else if (m_units == "METER") {
        scaleFactor = 1000.0;  // 1 meter = 1000 mm
    } else {
        LOG_WARNING("Unknown units '{}', assuming MM", m_units);
        m_units = "MM";
        return;
    }

    LOG_INFO("Converting units from {} to MM (scale: {})", m_units, scaleFactor);

    // Create scaling transformation
    gp_Trsf transform;
    transform.SetScale(gp_Pnt(0, 0, 0), scaleFactor);

    // Apply transformation
    BRepBuilderAPI_Transform transformer(m_shape, transform, Standard_True);
    m_shape = transformer.Shape();

    m_unitConverted = true;
    m_units = "MM";
}

int STEPReader::countSolids() const {
    if (m_shape.IsNull()) {
        return 0;
    }

    int count = 0;
    TopExp_Explorer exp(m_shape, TopAbs_SOLID);
    for (; exp.More(); exp.Next()) {
        count++;
    }

    return count;
}

TopoDS_Shape STEPReader::getShape() const {
    return m_shape;
}

TopoDS_Solid STEPReader::getSolid() const {
    if (m_shape.IsNull()) {
        return TopoDS_Solid();
    }

    // Return first solid found
    TopExp_Explorer exp(m_shape, TopAbs_SOLID);
    if (exp.More()) {
        return TopoDS::Solid(exp.Current());
    }

    return TopoDS_Solid();
}

std::string STEPReader::getPartName() const {
    return m_partName;
}

std::string STEPReader::getUnits() const {
    return m_units;
}

std::string STEPReader::getOriginalUnits() const {
    return m_originalUnits;
}

bool STEPReader::wasUnitConverted() const {
    return m_unitConverted;
}

int STEPReader::getSolidCount() const {
    return countSolids();
}

std::vector<TopoDS_Solid> STEPReader::getAllSolids() const {
    std::vector<TopoDS_Solid> solids;

    if (m_shape.IsNull()) {
        return solids;
    }

    TopExp_Explorer exp(m_shape, TopAbs_SOLID);
    for (; exp.More(); exp.Next()) {
        solids.push_back(TopoDS::Solid(exp.Current()));
    }

    return solids;
}

std::string STEPReader::getLastError() const {
    return m_lastError;
}

bool STEPReader::isValid() const {
    return !m_shape.IsNull();
}

std::string STEPReader::getAPVersion() const {
    return m_apVersion;
}

} // namespace openpanelcam

