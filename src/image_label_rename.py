import os
import glob

path = "/media/falcon/50fe2d19-4535-4db4-85fb-6970f063a4a11/ActiveDrive/HYD_2023/hyundai_temp_data"

labels = glob.glob(os.path.join(path, "*", "*", "label", "*.txt"))
labels.sort()
for label in labels:
    image = label.replace("label", "image").replace("txt", "png")
    rename_label = label.replace(label.split("/")[-1], label.split("/")[-3]+"_"+label.split("/")[-1])
    rename_image = image.replace(image.split("/")[-1], image.split("/")[-3]+"_"+image.split("/")[-1])
    os.rename(label, rename_label)
    os.rename(image, rename_image)

print()