_`food_volume_measure usage demonstration`
==========================================

:code:`food_volume_measure` measures food volume on oven trays from a fixed depth camera, using
the IM method (baseline-plane height-difference integral).

_`Theory`
---------

The empty-oven baseline is first turned into a height map on a RANSAC-fitted plane. The food
cloud is voxel-downsampled and clustered in that plane, and each grid cell keeps only its
highest food point. The volume is the per-cell height difference between food surface and
baseline, integrated over the cell area:

.. math::
   :label: im volume

   V = \sum_{(u,v)} \left( h_{\text{food}}(u,v) - h_{\text{baseline}}(u,v) \right) \cdot \Delta s^2

where :math:`\Delta s` is the integration resolution. Depth lost to reflections is filled in
conservatively inside a single foreground component. Filled cells and measured cells are counted
separately.

_`Usage demonstration`
----------------------

:code:`FoodVolumeMeasurer` wraps the whole pipeline. Set the empty-oven baseline and the food
frame, tune parameters with the chainable setters or a JSON file, then call :code:`run`. It
returns a :code:`VolumeEstimate` with the integrated volume (cm³), point counts, footprint area
and coverage, mean and max height, a per-component breakdown, and reference volumes (AABB, OBB,
convex hull).

.. code-block:: cpp
   :caption: IM measurement demo
   :name: pcd im measurement

   #include "measurement.hpp"

   int main() {
       PointCloud baseline = load_pcd("empty_oven.pcd");
       PointCloud food = load_pcd("food.pcd");
       FoodVolumeMeasurer measurer;
       VolumeEstimate est =
           measurer.set_baseline({baseline}).set_food(food).run();
       if (est.status != MeasurementStatus::kSuccess) {
           return 1;
       }
       // est.volume_cm3, est.raw_volume_cm3, est.interpolated_volume_cm3
       return 0;
   }

_`Parameter configuration`
--------------------------

The common parameters have chainable setters. Every field, including the hole-completion guards,
can also be loaded from or written to JSON:

.. code-block:: cpp
   :caption: chainable parameter setters
   :name: chainable setters

   FoodVolumeMeasurer measurer;
   measurer.set_voxel_size(0.003)
       .set_integration_resolution(0.003)
       .set_height_range(0.0015, 0.05)
       .set_cluster_params(0.010, 8)
       .set_plane_distance_threshold(0.003);
   measurer.load_config_from_json("measurement_config.json");