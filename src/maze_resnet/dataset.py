"""
Maze Signs Dataset Loader
This script defines a custom dataset class for loading images and labels from specified directories.

Group Members:
- Dyllon Preston
- Richi Dubey
"""


import os
import random
from PIL import Image
import torch
from torch.utils.data import Dataset
import torchvision.transforms as transforms

class MazeSignsDataset(Dataset):
    def __init__(self, sources, transform=None, split=None, split_ratio=(0.7, 0.0, 0.3), seed=None):
        """
        Args:
            sources (list of tuples): Each tuple is (label_file, image_dir).
            transform (callable, optional): Optional transform to be applied on an image.
            split (str, optional): Which split of the data to return. Should be one of 'train', 'val', or 'test'.
                                   If None, the full dataset is returned.
            split_ratio (tuple): A tuple of three floats indicating the fraction of data for (train, val, test).
                                 For example, (0.8, 0.1, 0.1).
            seed (int): Random seed for reproducibility.
        """
        self.data = []  # list of tuples (image_path, label)
        
        # Loop through each source folder and load the label file
        for label_file, image_dir in sources:
            with open(label_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) == 2:
                        img_name, label = parts
                        full_path = os.path.join(image_dir, img_name + '.png')
                        self.data.append((full_path, int(label)))
        
        # Shuffle the dataset for splitting
        if seed is not None:
            random.seed(seed)
            random.shuffle(self.data)

        # If a split is requested, create a subset of the data
        if split is not None:
            assert split in ['train', 'val', 'test'], "split must be 'train', 'val', or 'test'"
            total = len(self.data)
            train_end = int(total * split_ratio[0])
            val_end = train_end + int(total * split_ratio[1])
            
            if split == 'train':
                self.data = self.data[:train_end]
            elif split == 'val':
                self.data = self.data[train_end:val_end]
            elif split == 'test':
                self.data = self.data[val_end:]
        
        # Define default transformation if none provided.
        # Here, we compute the dataset's mean and std, then build a transform pipeline.
        if transform is None:
            computed_mean, computed_std = self.compute_mean_std()
            self.mean = computed_mean
            self.std = computed_std
            print(f"Computed mean: {computed_mean}, std: {computed_std}")
            self.transform = transforms.Compose([
                transforms.Resize((240, 320)),  # Resize to a fixed size
                # transforms.RandomCrop((240, 320), padding=4),
                # transforms.RandomHorizontalFlip(),
                # transforms.RandomVerticalFlip(),
                transforms.RandomAffine(degrees=0, translate=(0.15, 0.15)),
                # transforms.RandomResizedCrop((240, 320), scale=(0.8, 1.0)),
                transforms.RandomPerspective(distortion_scale=0.3, p=0.5, fill=0),
                transforms.RandomRotation(20),
                transforms.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.3, hue=0.1),
                transforms.ToTensor(),
                transforms.Normalize(mean=computed_mean, std=computed_std)
            ])
        else:
            self.transform = transform

    def compute_mean_std(self):
        """
        Computes per-channel mean and std over the dataset images.
        Note: This is a simple implementation that averages per-image statistics.
        """
        to_tensor = transforms.ToTensor()
        mean = torch.zeros(3)
        std = torch.zeros(3)
        n = len(self.data)
        for img_path, _ in self.data:
            image = Image.open(img_path).convert('RGB')
            tensor = to_tensor(image)  # tensor shape: [C, H, W]
            # Compute mean and std per channel for this image, then add to accumulators.
            mean += tensor.mean(dim=[1,2])
            std += tensor.std(dim=[1,2])
        mean /= n
        std /= n
        return mean.tolist(), std.tolist()

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        img_path, label = self.data[idx]
        image = Image.open(img_path).convert('RGB')  # Ensure image is in RGB format

        if self.transform:
            image = self.transform(image)

        return image, label

if __name__ == '__main__':
    # List of sources: (label_file, image_dir)
    sources = [
        ('./2024F_Gimgs/labels.txt', './2024F_Gimgs/'),
        ('./2024F_imgs/labels.txt', './2024F_imgs/'),
        ('./2025S_imgs/labels.txt', './2025S_imgs/'),
    ]
    
    # Create dataset instances for training, validation, and testing
    train_dataset = MazeSignsDataset(sources, split='train', seed=42)
    val_dataset = MazeSignsDataset(sources, split='val', seed=42)
    test_dataset = MazeSignsDataset(sources, split='test', seed=42)
    
    print("Total training samples:", len(train_dataset))
    print("Total validation samples:", len(val_dataset))
    print("Total test samples:", len(test_dataset))
    
    # Get a sample
    sample_img, sample_label = train_dataset[0]
    print("Sample label:", sample_label)
