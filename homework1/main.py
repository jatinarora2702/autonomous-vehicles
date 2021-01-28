import os
from shutil import copyfile

import numpy as np
import torch
from PIL import Image
from torchvision import models, transforms


def get_correct_count(preds, gold, mapping):
    preds = list(preds.cpu().numpy())
    gold = list(gold.cpu().numpy())
    corrects = 0
    for i in range(len(preds)):
        if preds[i] in mapping and mapping[preds[i]] == gold[i]:
            corrects += 1
    return corrects


# https://discuss.pytorch.org/t/imagenet-example-accuracy-calculation/7840
def accuracy(output, labels, topk=(1, 5)):
    """
    Computes the precision@k for the specified values of k
    output: Tensor [N, num_classes]
    labels: LongTensor [N]
    """
    maxk = max(topk)
    batch_size = labels.size(0)

    _, pred = output.topk(maxk, 1, True, True)
    pred = pred.t()
    correct = pred.eq(labels.view(1, -1).expand_as(pred))

    res = []
    for k in topk:
        correct_k = correct[:k].view(-1).float().sum(0, keepdim=True)
        res.append(correct_k.mul_(100.0 / batch_size).detach().cpu().numpy()[0])
    return res


def define_class_mapping():
    class_index = dict()
    with open("class_mapping.txt", "r") as f:
        for index, line in enumerate(f):
            class_index[line.strip()] = index
    return class_index


def load_img(image_path, transform):
    img = Image.open(image_path).convert('RGB')
    img = transform(img)
    return img


def eval_model(model, mapping, device, data_path):
    data_transforms = transforms.Compose([
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    model.eval()
    all_outputs = []
    all_labels = []

    for class_name in os.listdir(data_path):
        inputs = []
        labels = []
        for img in os.listdir(os.path.join(data_path, class_name)):
            inputs.append(load_img(os.path.join(data_path, class_name, img), data_transforms))
            labels.append(mapping[class_name])
        inputs = torch.stack(inputs).to(device)
        labels = torch.tensor(np.array(labels), device=device)
        with torch.no_grad():
            outputs = model(inputs)
            all_outputs.append(outputs)
            all_labels.append(labels)
        class_acc = accuracy(outputs, labels, topk=(1, 5))
        print(
            "Error({0}): Top-1: {1:.2f}, Top-5: {2:.2f}".format(class_name, 100.0 - class_acc[0], 100.0 - class_acc[1]))
    all_outputs = torch.cat(all_outputs, dim=0)
    all_labels = torch.cat(all_labels, dim=0)
    overall_acc = accuracy(all_outputs, all_labels, topk=(1, 5))
    print(
        "Error({0}): Top-1: {1:.2f}, Top-5: {2:.2f}".format("Overall", 100.0 - overall_acc[0], 100.0 - overall_acc[1]))


def build_val_set(mapping, imagenet_validation_dataset_dir):
    required_indices = []
    for class_name in os.listdir("pictures/test"):
        required_indices.append(mapping[class_name])

    pic_info = {}
    with open("ILSVRC2012_validation_ground_truth_labels.txt", "r") as f:
        for i, line in enumerate(f):
            class_index = int(line.strip())
            if class_index in required_indices:
                pic_info[i] = class_index

    rev_mapping = dict()
    for key, val in mapping.items():
        rev_mapping[val] = key

    curr = 0
    for pic_name in os.listdir(os.path.join(imagenet_validation_dataset_dir)):
        try:
            int(pic_name.split("_")[-1][:-5])
            if curr in pic_info:
                src = os.path.join(imagenet_validation_dataset_dir, pic_name)
                folder = os.path.join("pictures/val", rev_mapping[pic_info[curr]])
                if not os.path.exists(folder):
                    os.makedirs(folder)
                dst = os.path.join(folder, pic_name)
                copyfile(src, dst)
            curr += 1
        except:
            pass


def main():
    mapping = define_class_mapping()
    # build_val_set(mapping, imagenet_validation_dataset_dir="C:\\Users\\Jatin\\Downloads\\val")

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = models.resnet18(pretrained=True).to(device)

    eval_model(model, mapping, device, data_path="pictures/val")
    eval_model(model, mapping, device, data_path="pictures/test")


if __name__ == "__main__":
    main()
