from typing import Tuple
import numpy as np
from numpy.typing import NDArray
from torch import Tensor
import torchvision.transforms.functional as TF
import torchvision.transforms as T
from torch.utils.data import Dataset


class GraspDataset(Dataset):
    def __init__(self, train: bool=True) -> None:
        '''Dataset of successful grasps.  Each data point includes a 64x64
        top-down RGB image of the scene and a grasp pose specified by the gripper
        position in pixel space and rotation (either 0 deg or 90 deg)

        The datasets are already created for you, although you can checkout
        `collect_dataset.py` to see how it was made (this can take a while if you
        dont have a powerful CPU).
        '''
        mode = 'train' if train else 'val'
        self.train = train
        data = np.load(f'{mode}_dataset.npz')
        self.imgs = data['imgs']
        self.actions = data['actions']

    def transform_grasp(self, img: Tensor, action: np.ndarray) -> Tuple[Tensor, np.ndarray]:
        '''Randomly rotate grasp by 0, 90, 180, or 270 degrees.  The image can be
        rotated using `TF.rotate`, but you will have to do some math to figure out
        how the pixel location and gripper rotation should be changed.

        Arguments
        ---------
        img:
            float tensor ranging from 0 to 1, shape=(3, 64, 64)
        action:
            array containing (px, py, rot_id), where px specifies the row in
            the image (heigh dimension), py specifies the column in the image (width dimension),
            and rot_id is an integer: 0 means 0deg gripper rotation, 1 means 90deg rotation.

        Returns
        -------
        tuple of (img, action) where both have been transformed by random
        rotation in the set (0 deg, 90 deg, 180 deg, 270 deg)

        Note
        ----
        The gripper is symmetric about 180 degree rotations so a 180deg rotation of
        the gripper is equivalent to a 0deg rotation and 270 deg is equivalent to 90 deg.

        Example Action Rotations
        ------------------------
        action = (32, 32, 1)
         - Rot   0 deg : rot_action = (32, 32, 1)
         - Rot  90 deg : rot_action = (32, 32, 0)
         - Rot 180 deg : rot_action = (32, 32, 1)
         - Rot 270 deg : rot_action = (32, 32, 0)

        action = (0, 63, 0)
         - Rot   0 deg : rot_action = ( 0, 63, 0)
         - Rot  90 deg : rot_action = ( 0,  0, 1)
         - Rot 180 deg : rot_action = (63,  0, 0)
         - Rot 270 deg : rot_action = (63, 63, 1)
        '''
        ################################
        # Implement this function for Q4
        ################################

        # return img, action

        # Randomly select an angle from the list
        rot_angle = int(np.random.choice([0, 90, 180, 270]))

        # Rotate the image
        img = TF.rotate(img, rot_angle)

        # Action is (px, py, rot_id)
        px, py, rot_id = action

        # Get the height and width of the image
        _ , H, W = img.shape

        # print(f"H: {H}, W: {W}")

        # Rotate the action
        if rot_angle == 0:
            new_px, new_py = px, py

        elif rot_angle == 90:
            rot_id = 1 - rot_id # Switch between 0 and 1
            new_px, new_py = W - py - 1, px

        elif rot_angle == 180:
            new_px, new_py = H - px - 1, W - py - 1

        elif rot_angle == 270:
            rot_id = 1 - rot_id # Switch between 0 and 1
            new_px, new_py = py, H - px - 1


        # Return the rotated image and action
        action_new = np.array([new_px, new_py, rot_id])

        return img, action_new

    def __getitem__(self, idx: int) -> Tuple[Tensor, Tensor]:
        img = self.imgs[idx]
        action = self.actions[idx]

        H, W = img.shape[:2]
        img = TF.to_tensor(img)
        if np.random.rand() < 0.5:
            img = TF.rgb_to_grayscale(img, num_output_channels=3)

        if self.train:
            img, action = self.transform_grasp(img, action)

        px, py, rot_id = action
        label = np.ravel_multi_index((rot_id, px, py), (2, H, W))

        return img, label

    def __len__(self) -> int:
        '''Number of grasps within dataset'''
        return self.imgs.shape[0]
