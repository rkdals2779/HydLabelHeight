import numpy as np
import cv2


class PlaneEstim:
    def __init__(self, cfg):
        self.sections = []
        self.plane_height_thresh = 0.05
        self.min_section_size = 70
        self.standard_plane = np.array([-0.03025224, -0.99259721, 0.11762477, 2.05])
        # self.standard_plane = np.array([-0.03025224, -0.99259721,  0.11762477,  1.97306183])
        # self.standard_plane = np.array([0, -1, 0, 3.5])

    def fit_section_planes(self, pcd, label):
        """
        :param pcd: [[(N0,5), (N1,5), ...],   : first rows of sections
                     [(Ni,5), (Ni+1,5), ...], : second rows of sections
                    ]
        :return:
        """
        planes, label_out_pcd = self.estimate_planes(pcd, label)            #label_out_pcd => after pcd check delete
        # print("planes1", planes.reshape(3, -1))
        planes = self.update_standard_plane(planes)
        # print("planes2", planes.reshape(3, -1))
        planes = self.fill_empty_plane(planes)
        # print("planes3", planes.reshape(3, -1))
        planes = self.spatial_average(planes)
        # print("planes4", planes.reshape(3, -1))
        planes = self.temporal_average(planes)
        # print("planes5", planes.reshape(3, -1))
        return planes, label_out_pcd

    def estimate_planes(self, pcd_splits, label):
        """
        :param pcd_splits: [[(N0,5), (N1,5), ...],   : first rows of sections
                             [(Ni,5), (Ni+1,5), ...], : second rows of sections
                            ]
        :return: (rows, cols, 4) [nx, ny, nz, h], nx*x + ny*y + nz*z + h = 0
        """
        planes = []
        filtered_pcd = []                              #filtered_pcd => after pcd check delete
        for row_splits in pcd_splits:
            row_planes = []
            for section_pcd in row_splits:
                section_pcd = self.filter_out_label(section_pcd, label)
                plane = self.estimate_plane_impl(section_pcd)
                row_planes.append(plane)
                filtered_pcd.append(section_pcd)        #filtered_pcd => after pcd check delete
            planes.append(row_planes)
        planes = np.array(planes)
        return planes, filtered_pcd                      #filtered_pcd => after pcd check delete

    def filter_out_label(self, pcd, labels):
        if labels is None:
            return pcd

        for label in labels:
            xywh = label[1:5]
            mask = np.logical_or(np.logical_or((xywh[0] > pcd[:, 3]), (pcd[:, 3] > xywh[0] + xywh[2])),
                                 np.logical_or((xywh[1] > pcd[:, 4]), (pcd[:, 4] > xywh[1] + xywh[3]))
                                 )
            pcd = pcd[mask]
        return pcd

    def estimate_plane_impl(self, section_pcd):
        if section_pcd.shape[0] < 70:
            return np.zeros((4,), np.float64)

        pcd = section_pcd[:, :3]
        inlier_pcd = np.copy(pcd)
        for i in range(5):
            center = np.mean(inlier_pcd, axis=0)
            normal = self.estimate_normal(inlier_pcd)
            plane_height = np.dot(center, normal)
            point_height = np.abs(np.dot(normal, pcd.T) - plane_height)
            height_threshold = np.quantile(point_height, [0.9 - i * 0.1])
            #print("point_height", point_height)
            #print("threshold", height_threshold)
            inlier_pcd = pcd[point_height < height_threshold]

        center = np.mean(inlier_pcd, axis=0)
        normal = self.estimate_normal(inlier_pcd)
        plane_height = np.dot(center, normal)
        plane = np.concatenate([normal, [-plane_height]])
        point_height = np.abs(np.dot(normal, pcd.T) - plane_height)
        height_90per = np.quantile(point_height, [0.5])
        plane_valid = False
        if ((height_90per < self.plane_height_thresh) and
                ((np.sum(point_height < self.plane_height_thresh)) > self.min_section_size)):
            plane_valid = True

        if self.standard_plane is not None:
            if (np.dot(self.standard_plane[:3], plane[:3]) < np.cos(np.pi / 9)
                    or np.abs(self.standard_plane[3] - plane[3]) > 0.2):
                plane_valid = False

        return plane * float(plane_valid)

    def estimate_normal(self, pcd):
        center = np.mean(pcd, axis=0)
        diff = pcd - center  # (N, 3)
        scatter = np.dot(diff.T, diff)  # (3, 3)
        eigval, eigvec = np.linalg.eig(scatter)
        minidx = np.argmin(eigval)
        normal = eigvec[:, minidx]
        return normal

    def update_standard_plane(self, planes):
        non_empty_mask = planes[:, :, 3] != 0
        valid_planes = planes[non_empty_mask, :]
        num_valid = np.sum(valid_planes)
        # TODO: compare standard plane with planes
        if self.standard_plane is None:
            if num_valid < 3:
                return planes
            else:
                self.standard_plane = np.mean(valid_planes, axis=0)
        else:
            cos_val = np.dot(planes[..., :3], self.standard_plane[:3])
            angles = np.arccos(cos_val)
            valid_mask = angles < np.pi/15
            planes = planes * valid_mask[..., np.newaxis]
            non_empty_mask = planes[:, :, 3] != 0
            valid_planes = planes[non_empty_mask, :]
            cat_planes = np.concatenate([valid_planes, [self.standard_plane]], axis=0)
            self.standard_plane = np.mean(cat_planes, axis=0)
        self.standard_plane[:3] /= np.sqrt(np.dot(self.standard_plane[:3], self.standard_plane[:3]))
        # print('standard plane', self.standard_plane)
        return planes

    def fill_empty_plane(self, planes):
        """
        :param planes: (rows, cols, 4)
        :return:
        """
        if self.standard_plane is not None:
            empty_mask = planes[:, :, 3] == 0
            planes[empty_mask, :] = self.standard_plane
        return planes

    def spatial_average(self, planes):
        rows, cols = planes.shape[:2]
        planes_pad = np.pad(planes, pad_width=((1,1), (1,1), (0,0)), mode='constant', constant_values=0)
        non_zeros = planes_pad[..., 3:4] != 0
        planes_list = []
        non_zeros_list = []
        weights = np.array([[0.5, 5, 0.5], [5, 7, 5], [0.5, 5, 0.5]])
        weights /= np.sum(weights)
        for y in range(3):
            for x in range(3):
                planes_list.append(planes_pad[y:y+rows, x:x+cols, :] * weights[y, x])
                non_zeros_list.append(non_zeros[y:y+rows, x:x+cols, :] * weights[y, x])

        planes_list = np.stack(planes_list, axis=0)         # (9,2,3,4)
        non_zeros_list = np.stack(non_zeros_list, axis=0)   # (9,2,3,4)
        planes_smooth = np.sum(planes_list, axis=0) / np.sum(non_zeros_list, axis=0)    # (2,3,4)
        planes_smooth[..., :3] /= np.sqrt(np.sum(planes_smooth[..., :3] * planes_smooth[..., :3], axis=-1, keepdims=True))
        return planes_smooth

    def temporal_average(self, planes):
        std_weight = 1
        planes += self.standard_plane * std_weight
        planes /= 1 + std_weight
        planes[..., :3] /= np.sqrt(np.sum(planes[..., :3] * planes[..., :3], axis=-1, keepdims=True))
        return planes
