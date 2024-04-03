import cv2
import numpy as np
import os.path as op

from glob import glob

import config as cfg


class Dataset:
    def __init__(self, dataset):
        self.frame_list = glob(op.join(dataset["path"], "label", "*.txt"))
        # frame_ids = [int(op.basename(file).split('.')[0][5:]) for file in frame_list]
        # frame_ids = np.array(frame_ids)
        # frame_ids = np.concatenate([frame_ids, frame_ids-1, frame_ids-2], axis=0)
        # frame_ids = np.sort(np.unique(frame_ids), axis=0)
        # self.frame_list = [op.join(op.dirname(frame_list[0]), f'frame{frame_id:06d}.txt') for frame_id in frame_ids]
        self.frame_list.sort()
        self.cur_label_path = None

    def num_frame(self):
        return len(self.frame_list)

    def get_frame(self, index):
        # return image, pcd, label
        label_name = self.frame_list[index]
        print(f"NUM: {index} Label name: {label_name}")
        image, pcd, label = None, None, None

        image_name = label_name.replace('label', 'image').replace('txt', 'png')
        if op.isfile(image_name):
            image = cv2.imread(image_name)

        pcd_name = label_name.replace('label', 'pcd').replace('txt', 'npz')
        if op.isfile(pcd_name):
            pcd = self.get_pcd(pcd_name)

        if op.isfile(label_name):
            label = self.get_label(label_name)
        self.cur_label_path = label_name
        # label = [['one', 500, 800, 1469, 1200, 0.5]]
        return image, pcd, label

    def get_pcd(self, pcd_name):
        pcd = np.load(pcd_name)
        pcd = pcd["arr_0"]
        return pcd.astype(np.float64)

    def get_label(self, label_name):
        label_info = []
        with open(label_name, 'r') as f:
            label_lines = f.readlines()
            for label_line in label_lines:
                label_line = label_line.strip().split(", ")
                label_info_per_line = self.get_bbox(label_line)
                label_info.append(label_info_per_line)

        return label_info

    def get_bbox(self, label):
        category = label[0]
        # category_id = self.convert_category2id(category) ----> config?
        x, y, w, h, height = (int(float(label[2])), int(float(label[1])),
                              int(float(label[4])), int(float(label[3])), float(label[5]))
        label_info = [category, x, y, w, h, height]

        return label_info

    def update_label(self, label_anno):
        if label_anno is None:
            print("[update_label] label is None")
            return
        updated_label = []
        depth = [d[-1] for d in label_anno]
        label_name = self.cur_label_path
        with open(label_name, 'r') as f:
            labels = f.read()
        instances = labels.split('\n')[:-1]
        for instance, d in zip(instances, depth):
            cate_bbox = instance.split(', ')[:-1]
            updated_instance = cate_bbox + [d]
            updated_instance = ", ".join(map(str, updated_instance))
            updated_label.append(updated_instance)
        updated_label = "\n".join(map(str, updated_label)) + "\n"
        with open(label_name, 'w') as f:
            f.writelines(updated_label)


if __name__ == '__main__':
    path = cfg.DATASET["path"]
    dataset = Dataset(path)
