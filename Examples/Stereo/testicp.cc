#include "Thirdparty/robust_pcl_registration/robust_pcl_registration/pda.h"
#include<System.h>

int main(int argc, char **argv)
{
    // Load the IPDA parameters.
    IpdaParameters ipda_params;
    ipda_params.save_aligned_cloud = true;
    ipda_params.solver_minimizer_progress_to_stdout = false;
    ipda_params.solver_use_nonmonotonic_steps = true;
    ipda_params.use_gaussian = true;
    ipda_params.visualize_clouds = true;
    ipda_params.dof = 100.0;
    ipda_params.point_size_aligned_source = 3.0;
    ipda_params.point_size_source = 3.0;
    ipda_params.point_size_target = 3.0;
    ipda_params.radius = 0.5;
    ipda_params.solver_function_tolerance = 1.0e-16;
    ipda_params.source_filter_size = 5.0;
    ipda_params.target_filter_size = 0.0;
    ipda_params.transformation_epsilon = 1.0e-3;
    ipda_params.dimension = 3;
    ipda_params.maximum_iterations = 100;
    ipda_params.max_neighbours = 20;
    ipda_params.solver_maximum_iterations = 100;
    ipda_params.solver_num_threads = 8;
    ipda_params.aligned_cloud_filename = "aligned.pcd";
    ipda_params.frame_id = "map";
    ipda_params.source_cloud_filename = "source.pcd";
    ipda_params.target_cloud_filename = "target.pcd";

    Ipda ipda(ipda_params);

    // Create in and out clouds

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(ipda_params.source_cloud_filename, *cloud_in);
    pcl::io::loadPCDFile<pcl::PointXYZ>(ipda_params.target_cloud_filename, *cloud_out);

    ipda.evaluate(cloud_in, cloud_out);


}     
