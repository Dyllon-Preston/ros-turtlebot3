# Maze Sign Classification

This project implements classifiers for maze signs using a CNN approach. The system can identify six different sign types: empty wall, left, right, do not enter, stop, and goal.

## Project Structure

- dataset.py - Custom PyTorch dataset loader for maze sign images
- model.py - CNN model implementation using ResNet architecture
- train.py - Training script for the CNN model
- model_grader.py - Evaluation script for the trained model

## Dataset

The project uses multiple image datasets:
- 2024F_Gimgs - First dataset with labeled maze sign images
- 2024F_imgs - Second dataset with labeled maze sign images
- 2025S_imgs - Third dataset with labeled maze sign images

Each dataset contains a labels.txt file mapping image filenames to class labels (0-5).

## Training the CNN Model

To train the CNN model:

```bash
python train.py
```

This will:
1. Load images from all datasets
2. Split data into training and testing sets
3. Train a ResNet-based CNN model
4. Save the model to maze_sign_cnn.pth
5. Print training and test accuracy for each epoch

## Testing the Trained CNN Model

To evaluate a trained model:

```bash
python3 model_grader.py --data_path ./2024F_Gimgs/ --model_path saved_models/maze_sign_cnn_final.pth
```

Arguments:
- `--data_path`: Path to validation dataset
- `--model_path`: Path to trained model file

## Requirements

See requirements.txt
