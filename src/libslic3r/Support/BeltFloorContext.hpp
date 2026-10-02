#pragma once

#include "../libslic3r.h"
#include "../Point.hpp"
#include "../Polygon.hpp"
#include "../Slicing.hpp"
#include "../PrintConfig.hpp"

namespace Slic3r {

class PrintObject;

// Belt floor context: encapsulates the parameters and polygon computation
// for belt printer floor clipping in support generation.
//
// All belt floor code across SupportMaterial, TreeSupport, TreeSupport3D,
// and TreeModelVolumes uses the same 4 parameters and the same 4-case
// polygon construction.  This class consolidates that logic.
//
// Construct once per PrintObject, then call surface_polygon() or
// valid_region_polygon() per layer with the layer's print_z.
class BeltFloorContext
{
public:
    BeltFloorContext() = default;

    // Initialize from slicing parameters and print config.
    // Uses global Z coordinates (for SupportMaterial, non-organic TreeSupport).
    bool init(const SlicingParameters &sp, const PrintConfig &pcfg);

    // Initialize with a Z offset subtracted from z_shift.
    // Uses local Z coordinates (for TreeSupport3D, TreeModelVolumes organic pipeline).
    bool init_local(const SlicingParameters &sp, const PrintConfig &pcfg,
                    double global_z_offset);

    bool is_active() const { return m_active; }

    // Compute the belt-side half-plane polygon at a given print_z.
    // This is the region where the belt surface exists.
    Polygons surface_polygon(coordf_t print_z) const;

    // Compute the valid-region half-plane polygon at a given print_z.
    // This is the complement: the region where support is allowed.
    Polygons valid_region_polygon(coordf_t print_z) const;

    // Compute the belt floor Z position at a given XY position (in slicing coords).
    // Returns -infinity if not active.
    double floor_print_z(const Point &pos_slicing) const;

    // The from_axis coordinate (unscaled, slicing frame) where the belt surface
    // crosses a horizontal plane at print_z.  Inverse of floor_print_z() along
    // the shear axis.  Only meaningful when is_active().
    coordf_t cutoff_u(coordf_t print_z) const
        { return (print_z - m_z_shift - m_floor_offset) / m_shear_factor; }

    // Pre-compute belt floor polygons for a range of layers.
    // layer_print_z(i) returns the print_z for layer index i.
    std::vector<Polygons> compute_per_layer_floors(
        size_t num_layers,
        const std::function<double(size_t)> &layer_print_z) const;

    // Accessors
    double shear_factor()  const { return m_shear_factor; }
    int    from_axis()     const { return m_from_axis; }
    double z_shift()       const { return m_z_shift; }
    double floor_offset()  const { return m_floor_offset; }

private:
    bool   m_active       = false;
    double m_shear_factor = 0.0;
    int    m_from_axis    = 1;  // 0=X, 1=Y
    double m_z_shift      = 0.0;
    double m_floor_offset = 0.0;

    // Internal: compute the raw half-plane polygon.
    // If belt_surface=true, returns the belt side; otherwise the valid (complement) side.
    Polygons half_plane(coordf_t print_z, bool belt_surface) const;
};

// Number of virtual "belt raft" layers the organic tree-support pipeline
// prepends below a belt-printer object so branches can grow down to the
// tilted belt surface instead of stopping at the object's first layer; 0 when
// that mode isn't active. `extra_depth` (mm) deepens it beyond the default.
// TreeModelVolumes (anti_overhang shift and m_raft_layers) and
// generate_support_areas() all index the same combined raft+object layer
// space, so they must all use one count from here -- a mismatch silently
// shifts one side's per-layer data against the other's.
size_t belt_organic_raft_layer_count(const PrintObject &print_object, double extra_depth = 0.);

} // namespace Slic3r
