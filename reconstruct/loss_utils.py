#
# This file is part of https://github.com/JingwenWang95/DSP-SLAM
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
#

import time
import torch
import numpy as np


def get_rays(sampled_pixels, invK):
    """
    This function computes the ray directions given sampled pixel
    and camera intrinsics
    :param sampled_pixels: (N, 2), order is [u, v]
    :param invK: (3, 3)
    :return: ray directions (N, 3) under camera frame
    """
    n = sampled_pixels.shape[0]
    # (n, 3) = (n, 2) (n, 1)
    u_hom = np.concatenate([sampled_pixels, np.ones((n, 1))], axis=-1)
    # (n, 3) = (n, 1, 3) * (3, 3)
    directions = (u_hom[:, None, :] * invK).sum(-1)

    return directions.astype(np.float32)

def get_time():
    """
    :return: get timing statistics
    """
    torch.cuda.synchronize()
    return time.time()
