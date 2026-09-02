"""RGB-D 桌面、物件幾何與抓取候選分析。"""

from .geometry_pipeline import analyze_rgbd_geometry, load_perception_spec

__all__ = ["analyze_rgbd_geometry", "load_perception_spec"]
