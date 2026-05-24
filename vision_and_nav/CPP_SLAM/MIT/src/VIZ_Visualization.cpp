#include "VIZ_Visualization.hpp"
#include "LG_Logging.hpp"
#include "OB_Observations.hpp"
#include "PROJ_ProjectiveUtils.hpp"
#include "VW_Views.hpp"

void __VIZ_WriteCamerasColmap(struct ViewSet vs, FILE *fp);
void __VIZ_WritePointsColmap(struct ObservationSet os, struct PointSet ps, struct ViewSet vs, FILE *fp);
void __VIZ_WriteImagesColmap(struct ObservationSet os, struct ViewSet vs, FILE *fp);

void VIZ_WriteColmap(struct ObservationSet os, struct PointSet ps,
                     struct ViewSet vs, std::string path) 
{
    const std::string cam_path = path + "cameras.bin";
    const std::string image_path = path + "images.bin";
    const std::string point_path = path + "points3D.bin";
    FILE *fp_point = fopen(point_path.c_str(), "wb+");
    if (fp_point == NULL) 
    {
        perror("Failed to open file");
        LG_Log("[ERROR] Failed to open file %s\n", point_path.c_str());
        return;
    }
    __VIZ_WritePointsColmap(os, ps, vs, fp_point);
    fclose(fp_point);

    FILE *fp_cam = fopen(cam_path.c_str(), "wb+");
    if (fp_cam == NULL) 
    {
        perror("Failed to open file");
        LG_Log("[ERROR] Failed to open file %s\n", cam_path.c_str());
        return;
    }
    __VIZ_WriteCamerasColmap(vs, fp_cam);
    fclose(fp_cam);

    FILE *fp_image = fopen(image_path.c_str(), "wb+");
    if (fp_image == NULL) 
    {
        perror("Failed to open file");
        LG_Log("[ERROR] Failed to open file %s\n", image_path.c_str());
        return;
    }
    __VIZ_WriteImagesColmap(os, vs, fp_image);
    fclose(fp_image);
}

u64 w, h;

void __VIZ_WriteCamerasColmap(struct ViewSet vs, FILE *fp) 
{
    LG_Log("[__VIZ_WriteCamerasColmap] Writing cameras colmap\n");
    const u64 num_cameras = static_cast<u64>(vs.views.size());
    LG_Log("[__VIZ_WriteCamerasColmap] num cameras = %llu\n", num_cameras);
    fwrite(&num_cameras, sizeof(u64), 1, fp);

    const u64 wh[2] = {w, h};
    CameraIntrinsics *ci = vs.views[0].intrinsics;
    const fp64 params[4] = {ci->K(0, 0), ci->K(1, 1), ci->K(0, 2), ci->K(1, 2)};
    for (u64 i = 0; i < num_cameras; i++) 
    {
        const i32 j = static_cast<i32>(i);
        fwrite(&j, sizeof(i32), 1, fp);

        const i32 model_id = MODEL_ID;
        fwrite(&model_id, sizeof(i32), 1, fp);
        fwrite(wh, sizeof(u64), 2, fp);
        fwrite(params, sizeof(fp64), 4, fp);
    }
}

void __VIZ_WritePointsColmap(struct ObservationSet os, struct PointSet ps, struct ViewSet vs, FILE *fp) 
{
    LG_Log("[__VIZ_WritePointsColmap] Writing points colmap\n");
    // dont care
    const fp64 error = 0.0f;
    u8 RGB[3] = {0, 0, 0};

    const u64 num_points = static_cast<u64>(ps.points.size());
    fwrite(&num_points, sizeof(u64), 1, fp);
    LG_Log("[__VIZ_WritePointsColmap] num points colmap = %llu\n", num_points);
    const size_t num_imgs = vs.views.size();
    LG_Log("[__VIZ_WritePointsColmap] Getting RGB images\n");
    std::vector<cv::Mat> imgs;
    imgs.resize(num_imgs);
    for(size_t i = 0; i < num_imgs; i++)
    {
        std::string path = "./colmap/images/frame" + std::to_string(i) + ".png";
        LG_Log("[__VIZ_WritePointsColmap] Getting RGB image %s\n", path.c_str());
        LG_Log("[__VIZ_WritePointsColmap] Getting RGB image index %d\n", i);
        imgs[i] = cv::imread(path, cv::IMREAD_COLOR_RGB);
    }
    w = imgs[0].cols;
    h = imgs[0].rows;
    LG_Log("[__VIZ_WritePointsColmap] RGB images have (w, h) = (%lld, %lld)\n", w, h);
    LG_Log("[__VIZ_WritePointsColmap] got all RGB images\n");
    for (u64 i = 0; i < num_points; i++) 
    {
        fwrite(&i, sizeof(u64), 1, fp);

        Eigen::Vector3d point_cart = PROJ_Homog2Cart(ps.points[i]);
        fwrite(point_cart.data(), sizeof(fp64), 3, fp);
        const size_t num_obs = ps.observations_indexes[i].size();
        fp64 R = 0.0f;
        fp64 G = 0.0f;
        fp64 B = 0.0f;
        for(size_t j = 0; j < num_obs; j++)
        {
            cv::Mat img_rgb = imgs[os.view_indexes[ps.observations_indexes[i][j]]];
            cv::Point2d obs = os.observations[ps.observations_indexes[i][j]];
            i64 x = static_cast<i64>(round(obs.x));
            i64 y = static_cast<i64>(round(obs.y));
            //LG_Log("[__VIZ_WritePointsColmap] getting pixel (y, x) = (%lld, %lld)\n", y, x);
            cv::Vec3b rgb = img_rgb.at<cv::Vec3b>(y, x);
            R += static_cast<fp64>(rgb[0]);
            G += static_cast<fp64>(rgb[1]);
            B += static_cast<fp64>(rgb[2]);
        }
        R /= num_obs;
        G /= num_obs;
        B /= num_obs;

        RGB[0] = static_cast<u8>(R);
        RGB[1] = static_cast<u8>(G);
        RGB[2] = static_cast<u8>(B);
        fwrite(RGB, sizeof(u8), 3, fp);

        const auto &obs_idx = ps.observations_indexes[i];
        u64 track = static_cast<u64>(num_obs);

        fwrite(&error, sizeof(fp64), 1, fp);
        fwrite(&track, sizeof(u64), 1, fp);
        for (size_t j = 0; j < num_obs; j++) 
        {
            const i32 point_2d_idx = obs_idx[j];
            const i32 image_id = static_cast<i32>(os.view_indexes[point_2d_idx]);
            fwrite(&image_id, sizeof(i32), 1, fp);
            fwrite(&point_2d_idx, sizeof(i32), 1, fp);
        }
    }
}

void __VIZ_WriteImagesColmap(struct ObservationSet os, struct ViewSet vs, FILE *fp) 
{
    LG_Log("[__VIZ_WriteImagesColmap] Writing images colmap\n");
    u64 num_images = vs.views.size();
    fwrite(&num_images, sizeof(u64), 1, fp);
    LG_Log("[__VIZ_WriteImagesColmap] num images = %llu\n", num_images);

    for (u64 i = 0; i < num_images; i++) 
    {
        const i32 image_id = static_cast<i32>(i);
        fwrite(&image_id, sizeof(i32), 1, fp);

        Param* params = vs.views[i].p;

        //colmap expects camera to world!
        Eigen::Matrix3d R_wc = params->q.toRotationMatrix();
        Eigen::Vector3d C_w  = params->t;

        Eigen::Matrix3d R_cw = R_wc.transpose();
        Eigen::Vector3d t_cw = -R_cw * C_w;

        Eigen::Quaterniond q_cw(R_cw);
        q_cw.normalize();

        const fp64 quat[4] = {
            q_cw.w(),
            q_cw.x(),
            q_cw.y(),
            q_cw.z()
        };
        fwrite(quat, sizeof(fp64), 4, fp);
        fwrite(t_cw.data(), sizeof(fp64), 3, fp);

        // camera id, now same as image
        fwrite(&image_id, sizeof(i32), 1, fp);
        LG_Log("[__VIZ_WriteImagesColmap] image id = %d\n", image_id);

        const std::string img_name = vs.views[i].image_name;
        fwrite(img_name.c_str(), sizeof(char), img_name.size() + 1, fp);
        LG_Log("[__VIZ_WriteImagesColmap] image name = %s\n", img_name.c_str());

        const u64 num_2d_points = vs.observations_indexes[i].size();
        fwrite(&num_2d_points, sizeof(fp64), 1, fp);

        const auto &obs_idxs = vs.observations_indexes[i];
        for (u64 j = 0; j < num_2d_points; j++) 
        {
            const u64 idx = obs_idxs[j];
            const cv::Point2d p2d = os.observations[idx];
            const fp64 p2d_raw[2] = {p2d.x, p2d.y};
            fwrite(&p2d_raw, sizeof(fp64), 2, fp);

            i64 id3D = os.point_indexes[idx];
            fwrite(&id3D, sizeof(i64), 1, fp);
        }
    }
}
