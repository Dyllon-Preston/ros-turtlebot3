"""

This script trains a ResNet model on the Maze Signs dataset.
The dataset consists of images of signs and their corresponding labels.

Group Members:
- Dyllon Preston
- Richi Dubey

"""

import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from dataset import MazeSignsDataset
from model import MazeSignResNet

# ANSI escape codes for coloring and styling
RESET = "\033[0m"
BOLD = "\033[1m"
UNDERLINE = "\033[4m"
BLUE = "\033[34m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"

def train():
    # -------------------
    # Config
    # -------------------
    sources = [
        ('./2024F_Gimgs/labels.txt', './2024F_Gimgs/'),
        ('./2024F_imgs/labels.txt', './2024F_imgs/'),
        ('./2025S_imgs/labels.txt', './2025S_imgs/'),
    ]

    use_val = False            # Use validation split
    split_ratio = (0.7, 0.0, 0.3)  # Train, Val, Test split ratios
    num_classes = 6

    batch_size = 32
    epochs = 300
    log_interval = 10         # Print every N batches
    save_interval = 1         # Save every N epochs
    save_dir = "saved_models"
    use_cuda = True
    seed = 42

    # Label names mapping
    label_names = {
        0: "empty wall",
        1: "left",
        2: "right",
        3: "do not enter",
        4: "stop",
        5: "goal"
    }

    # -------------------
    # Setup
    # -------------------
    torch.manual_seed(seed)
    device = torch.device("cuda" if torch.cuda.is_available() and use_cuda else "cpu")

    train_dataset = MazeSignsDataset(sources, split='train', seed=seed, split_ratio=split_ratio)
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4)

    if use_val:
        val_dataset = MazeSignsDataset(sources, split='val', seed=seed, split_ratio=split_ratio)
        val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4)

    # Always create a test dataset
    test_dataset = MazeSignsDataset(sources, split='test', seed=seed, split_ratio=split_ratio)
    test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False, num_workers=4)

    computed_mean = train_dataset.mean
    computed_std = train_dataset.std

    # Save the computed mean and std to a file
    mean_std_path = os.path.join(save_dir, "mean_std.txt")
    os.makedirs(save_dir, exist_ok=True)
    with open(mean_std_path, "w") as f:
        f.write(f"Mean: {computed_mean}\n")
        f.write(f"Std: {computed_std}\n")

    model = MazeSignResNet(num_classes=num_classes).to(device)
    criterion = nn.CrossEntropyLoss(label_smoothing=0.1)
    optimizer = torch.optim.SGD(model.parameters(), lr=0.1, momentum=0.9, weight_decay=5e-4)
    scheduler = torch.optim.lr_scheduler.OneCycleLR(
        optimizer, max_lr=0.1,
        steps_per_epoch=len(train_loader),
        epochs=epochs
    )

    print(f"{BOLD}{UNDERLINE}{CYAN}Starting Training...{RESET}\n")

    # -------------------
    # Training Loop
    # -------------------
    for epoch in range(1, epochs + 1):
        model.train()
        running_loss = 0.0
        correct = 0
        total = 0

        # For per-category training accuracy
        train_correct_by_class = {i: 0 for i in range(num_classes)}
        train_total_by_class = {i: 0 for i in range(num_classes)}

        for batch_idx, (images, labels) in enumerate(train_loader):
            images, labels = images.to(device), labels.to(device)

            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            scheduler.step()

            running_loss += loss.item() * images.size(0)
            _, preds = torch.max(outputs, 1)
            total += labels.size(0)
            correct += (preds == labels).sum().item()

            # Update per-category counts
            for pred, label in zip(preds, labels):
                label_val = label.item()
                train_total_by_class[label_val] += 1
                if pred.item() == label_val:
                    train_correct_by_class[label_val] += 1

            if (batch_idx + 1) % log_interval == 0:
                print(f"{YELLOW}[{BOLD}Epoch {epoch}/{epochs}{RESET}{YELLOW}] Step {batch_idx + 1}/{len(train_loader)} - Loss: {loss.item():.4f}{RESET}")

        train_loss = running_loss / total
        train_acc = 100.0 * correct / total
        print(f"\n{BOLD}{BLUE}Epoch {epoch} Training Summary:{RESET}")
        print(f"{GREEN}Training Loss: {train_loss:.4f}{RESET}, {GREEN}Overall Accuracy: {train_acc:.2f}%{RESET}")
        print(f"{BOLD}Training accuracy breakdown by category:{RESET}")
        for i in range(num_classes):
            if train_total_by_class[i] > 0:
                acc = 100.0 * train_correct_by_class[i] / train_total_by_class[i]
            else:
                acc = 0.0
            print(f"  {CYAN}{label_names[i]}:{RESET} {acc:.2f}%")

        # -------------------
        # Validation (if available)
        # -------------------
        if use_val:
            model.eval()
            val_loss = 0.0
            val_correct = 0
            val_total = 0
            val_correct_by_class = {i: 0 for i in range(num_classes)}
            val_total_by_class = {i: 0 for i in range(num_classes)}
            with torch.no_grad():
                for images, labels in val_loader:
                    images, labels = images.to(device), labels.to(device)
                    outputs = model(images)
                    loss = criterion(outputs, labels)
                    val_loss += loss.item() * images.size(0)
                    _, preds = torch.max(outputs, 1)
                    val_total += labels.size(0)
                    val_correct += (preds == labels).sum().item()

                    for pred, label in zip(preds, labels):
                        label_val = label.item()
                        val_total_by_class[label_val] += 1
                        if pred.item() == label_val:
                            val_correct_by_class[label_val] += 1

            if val_total > 0:
                val_loss /= val_total
                val_acc = 100.0 * val_correct / val_total
                print(f"{BOLD}{BLUE}Epoch {epoch} Validation Summary:{RESET}")
                print(f"{GREEN}Validation Loss: {val_loss:.4f}{RESET}, {GREEN}Overall Accuracy: {val_acc:.2f}%{RESET}")
                print(f"{BOLD}Validation accuracy breakdown by category:{RESET}")
                for i in range(num_classes):
                    if val_total_by_class[i] > 0:
                        acc = 100.0 * val_correct_by_class[i] / val_total_by_class[i]
                    else:
                        acc = 0.0
                    print(f"  {CYAN}{label_names[i]}:{RESET} {acc:.2f}%")
            else:
                print(f"{RED}Validation set is empty — skipping validation.{RESET}")

        # -------------------
        # Test Evaluation
        # -------------------
        model.eval()
        test_loss = 0.0
        test_correct = 0
        test_total = 0
        test_correct_by_class = {i: 0 for i in range(num_classes)}
        test_total_by_class = {i: 0 for i in range(num_classes)}
        with torch.no_grad():
            for images, labels in test_loader:
                images, labels = images.to(device), labels.to(device)
                outputs = model(images)
                loss = criterion(outputs, labels)
                test_loss += loss.item() * images.size(0)
                _, preds = torch.max(outputs, 1)
                test_total += labels.size(0)
                test_correct += (preds == labels).sum().item()

                for pred, label in zip(preds, labels):
                    label_val = label.item()
                    test_total_by_class[label_val] += 1
                    if pred.item() == label_val:
                        test_correct_by_class[label_val] += 1

        if test_total > 0:
            test_loss /= test_total
            test_acc = 100.0 * test_correct / test_total
            print(f"{BOLD}{BLUE}Epoch {epoch} Test Summary:{RESET}")
            print(f"{GREEN}Test Loss: {test_loss:.4f}{RESET}, {GREEN}Overall Accuracy: {test_acc:.2f}%{RESET}")
            print(f"{BOLD}Test accuracy breakdown by category:{RESET}")
            for i in range(num_classes):
                if test_total_by_class[i] > 0:
                    acc = 100.0 * test_correct_by_class[i] / test_total_by_class[i]
                else:
                    acc = 0.0
                print(f"  {CYAN}{label_names[i]}:{RESET} {acc:.2f}%")
        else:
            print(f"{RED}Test set is empty — skipping test evaluation.{RESET}")

        # -------------------
        # Save model
        # -------------------
        if epoch % save_interval == 0:
            os.makedirs(save_dir, exist_ok=True)
            save_path = os.path.join(save_dir, f"maze_sign_cnn.pth")
            torch.save(model.state_dict(), save_path)
            print(f"{YELLOW}Model saved to {save_path}{RESET}\n")

if __name__ == "__main__":
    train()
