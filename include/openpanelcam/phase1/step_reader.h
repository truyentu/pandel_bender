#pragma once

/**
 * @file step_reader.h
 * @brief STEP file reader for OpenPanelCAM
 *
 * Reads STEP files (ISO 10303) and extracts geometry:
 * - Supports AP203, AP214, AP242
 * - Auto-converts units to MM
 * - Validates shape topology
 * - Extracts metadata (part name, units, etc.)
 */

#include <openpanelcam/core/types.h>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <string>
#include <vector>

namespace openpanelcam {

/**
 * @brief STEP file reader class
 *
 * Usage:
 * @code
 *   STEPReader reader;
 *   if (reader.load("part.step")) {
 *       TopoDS_Shape shape = reader.getShape();
 *       // Process shape...
 *   } else {
 *       LOG_ERROR("Failed: {}", reader.getLastError());
 *   }
 * @endcode
 */
class STEPReader {
public:
    STEPReader();
    ~STEPReader();

    /**
     * @brief Load a STEP file
     * @param filepath Path to STEP file
     * @return true if successful, false on error
     *
     * Process:
     * 1. Read STEP file with STEPControl_Reader
     * 2. Validate shape topology
     * 3. Convert units to MM if needed
     * 4. Extract metadata
     *
     * @throws Phase1Exception if file not found or parse error
     */
    bool load(const std::string& filepath);

    /**
     * @brief Get the loaded shape
     * @return The complete shape (may be compound, solid, or assembly)
     * @pre load() returned true
     */
    TopoDS_Shape getShape() const;

    /**
     * @brief Get first solid from shape
     * @return First solid found, or null if none
     * @pre load() returned true
     */
    TopoDS_Solid getSolid() const;

    /**
     * @brief Get part name from STEP metadata
     * @return Part name or empty string if not found
     */
    std::string getPartName() const;

    /**
     * @brief Get units from STEP file
     * @return "MM", "INCH", or "UNKNOWN"
     */
    std::string getUnits() const;

    /**
     * @brief Get original units before conversion
     * @return Original units string
     */
    std::string getOriginalUnits() const;

    /**
     * @brief Check if unit conversion was performed
     * @return true if INCH → MM conversion happened
     */
    bool wasUnitConverted() const;

    /**
     * @brief Get number of solids in shape
     * @return Count of TopoDS_Solid objects
     */
    int getSolidCount() const;

    /**
     * @brief Get all solids from shape
     * @return Vector of all solids found
     */
    std::vector<TopoDS_Solid> getAllSolids() const;

    /**
     * @brief Get last error message
     * @return Error message or empty if no error
     */
    std::string getLastError() const;

    /**
     * @brief Check if shape is valid
     * @return true if loaded and valid
     */
    bool isValid() const;

    /**
     * @brief Get STEP application protocol version
     * @return "AP203", "AP214", "AP242", or "UNKNOWN"
     */
    std::string getAPVersion() const;

private:
    // Loaded data
    TopoDS_Shape m_shape;
    std::string m_filepath;
    std::string m_partName;
    std::string m_units;
    std::string m_originalUnits;
    std::string m_apVersion;
    bool m_unitConverted;
    std::string m_lastError;

    // Internal methods
    bool validateShape();
    void extractMetadata();
    void convertUnitsToMM();
    int countSolids() const;
};

} // namespace openpanelcam

