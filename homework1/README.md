# Homework 1

Pick any 10 ImageNet classes and pick 50 images from (1) ImageNet validation set, (2) image search results on Google or Bing. Compare the accuracy of a pretrained ResNet-18 model on these two and identify potential reasons for difference.

## Approach

We make use of pretrained [ResNet-18](https://pytorch.org/docs/stable/torchvision/models.html#torchvision.models.resnet18) model from TorchVision library. We pick the below 10 ImageNet classes:

* ```black stork```
* ```cougar```
* ```English setter```
* ```English springer```
* ```grey whale```
* ```kit fox```
* ```lesser panda```
* ```porcupine```
* ```sea lion```
* ```Siberian husky```

## Validation Set

We downloaded the standard ImageNet validation set from [here](https://academictorrents.com/details/5d6d0df7ed81efd49ca99ea4737e0ae5e3a5f2e5). The dataset has 50,000 images. Their ground-truth class labels are in file [```ILSVRC2012_validation_ground_truth_labels.txt```](ILSVRC2012_validation_ground_truth_labels.txt). The semantic class names corresponding to 0-indexed class labels are in file [```class_mapping.txt```](class_mapping.txt). The class mappings are courtesy of [this source](https://gist.github.com/yrevar/942d3a0ac09ec9e5eb3a).

## Test Set

We wrote a script, [```download_images.py```](download_images.py) to use the [cromedriver plugin](https://chromedriver.chromium.org/) and automatically search for images for a given query (class name) and download the image results.

## Running

```commandline
python main.py
```

## Observations

* We observe a general trend that validation set errors are lesser as compared to test set images. This is coherent with our belief that ImageNet validation set images may have a slightly different distribution as compared to images downloaded in an adhoc way from google search. Since the pre-trained model is tuned using the validation set so it does not perform that well on our test set.

* At the same time we see some similarities as well, like for Siberian Husky, top-1 error is 70% (Val) and 81.81% (Test). This shows that the validation set is a decently good representation and its deviation from real world images is not too much.
