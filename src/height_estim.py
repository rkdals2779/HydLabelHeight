import numpy as np


class HeightEstim:
    def __init__(self, cfg):
        self.threshold = cfg["category_threshold"]
        self.sections = []

    def estimate_heights(self, pcd, label, planes):
        pcd = self.append_height_to_pcd(pcd, planes)
        if label is not None:
            label = self.append_height_to_label(pcd, label)
        return pcd, label

    def append_height_to_pcd(self, pcd, planes):
        pcd_heights = []
        for row, (pcd_row, planes_row) in enumerate(zip(pcd, planes)):
            for col, (pcd_sec, plane_sec) in enumerate(zip(pcd_row, planes_row)):
                points_height = np.dot(pcd_sec[:, :3], plane_sec[:3])
                diff = points_height + plane_sec[3]
                pcd_heights.append(np.concatenate((pcd_sec, diff[:, np.newaxis]), axis=-1))
        pcd_heights = np.concatenate(pcd_heights, axis=0)
        return pcd_heights

    def append_height_to_label(self, pcd, label):
        updated_label = []
        for instance in label:
            category, x1, y1, w, h, height = instance
            # print("CATEGORY - thr", category, self.threshold[category])
            x2 = x1 + w
            y2 = y1 + h
            inlier_mask_hori = np.logical_and(pcd[:, 3] > x1, pcd[:, 3] < x2)
            inlier_mask_vert = np.logical_and(pcd[:, 4] > y1, pcd[:, 4] < y2)
            inlier_mask = np.logical_and(inlier_mask_hori, inlier_mask_vert)
            pcd_height = pcd[inlier_mask][:, -1]
            height_mean = np.mean(pcd_height)
            height_std = np.std(pcd_height)
            normalized_height = (pcd_height - height_mean) / height_std
            height_inlier = np.abs(normalized_height) < 2
            if pcd_height[height_inlier].size == 0:
                instance_height = 0
            else:
                instance_height = np.quantile(pcd_height[height_inlier], self.threshold[category])
                instance_height = round(instance_height, 3)
            updated_label.append([category, x1, y1, x2, y2, instance_height])
        return updated_label
