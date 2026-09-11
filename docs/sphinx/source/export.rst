_`food_volume_measure usage demonstration`
==========================================

Brief overview: the library :code:`food_volume_measure` measures food volume on
oven trays from a fixed depth camera, using the IM (baseline-plane
height-difference integral) method.

_`Theory`
---------

An empty-oven baseline is first discretized into a height map on a RANSAC-fitted
base plane. The food point cloud is voxel-downsampled and clustered in the base
plane, and every grid cell keeps only the highest food surface point. The volume
is the per-cell height difference between the food surface and the empty-oven
baseline, integrated over the cell area:

.. math::
   :label: im volume

   V = \sum_{(u,v)} \left( h_{\text{food}}(u,v) - h_{\text{baseline}}(u,v) \right) \cdot \Delta s^2

where :math:`\Delta s` is the integration resolution. Missing depth caused by
reflections is completed conservatively inside single foreground components;
completed cells are counted separately from measured cells.

_`Usage demonstration`
----------------------

The end-to-end :code:`VolumeMeasurement::measure` composes the atomic stages. It
returns a :code:`VolumeEstimate` with the integrated volume (cm³), point counts,
footprint area and coverage, mean/max height, and reference volumes (AABB, OBB,
convex hull).

.. code-block:: cpp
   :caption: IM measurement demo
   :name: pcd im measurement

   #include "volume_measurement.hpp"
   #include "volume_pointcloudprocess.hpp"

   int main() {
       vm::PointCloud baseline = vm::load_pcd("empty_oven.pcd");
       vm::PointCloud food = vm::load_pcd("food.pcd");
       vm::VolumeMeasurement measurement;
       vm::VolumeEstimate est =
           measurement.measure({baseline}, food);
       if (est.status != vm::MeasurementStatus::kSuccess) {
           return 1;
       }
       // est.volume_cm3, est.raw_volume_cm3, est.interpolated_volume_cm3
       return 0;
   }

_`Atomic pipeline stages`
-------------------------

The pipeline can also be driven stage by stage:

- :code:`vm::load_pcd` — load a PCD file into the PCL-free :code:`vm::PointCloud` model
- :code:`vm::preprocess_cloud` / :code:`vm::voxel_downsample` — unit normalization and voxel downsampling
- :code:`vm::remove_dominant_plane` — remove the dominant background plane(s)
- :code:`vm::build_baseline_model` — empty-oven baseline height map
- :code:`vm::extract_food_components` — foreground clustering and component selection
- :code:`vm::measure_component_volume` — height-difference integration with hole completion