def get_detectors(configs):
    if configs.detect_online:
        if configs.data_type == "KITTI":
            from .detector3d import get_detector3d
            return get_detector3d(configs)
    else:
        if configs.data_type == "KITTI":
            return None
        else:
            return None


def get_sequence(data_dir, configs):
    if configs.data_type == "KITTI":
        from .kitti_sequence import KITIISequence
        return KITIISequence(data_dir, configs)