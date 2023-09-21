# Project 1: Preliminary Configurations

To properly build PCL 1.11 and render PCL Viewer and related objects on Ubuntu 20.04 (UTM QEMU 7.0 aarch64), the following steps are required.

## Checklist

1. Compile and install `metslib` from source
2. Create symlinks to `vtk` and `pvtk`
3. Disable GPU acceleration from UTM's VM settings

## Configurations

### Compile and install `metslib` from source

Optimization toolkit [`metslib`](https://github.com/coin-or/metslib) is referenced by PCL's `CMakeLists.txt` but not available by default on Ubuntu 20.04. As far as I understand, this is more of a nice-to-have [1].

```bash
cd /home/$whoami/workspace/udacity-sfend/
wget https://www.coin-or.org/download/source/metslib/metslib-0.5.3.tgz
tar xzvf metslib-0.5.3.tgz
cd metslib-0.5.3
./configure --prefix=/usr/local
make
sudo make install
```

### Create symlinks to `vtk` and `pvtk`

In Ubuntu 20.04, `sudo apt install vtk7` places VTK 7.1 inside `vtk7` folder, but PCL looks for the files in `vtk`. For lack of a better alternative, its Python bindings `pvtk` are simply a symbolic link to Python 3 [1] [2].

```bash
sudo ln /usr/bin/vtk7 /usr/bin/vtk
sudo ln /usr/bin/python3 /usr/bin/pvtk
```

### Disable GPU acceleration from UTM's VM settings

Some `pcl::visualization` rendering properties such as `PCL_VISUALIZER_POINT_SIZE` or `PCL_VISUALIZER_LINE_LENGTH` are not compatible with UTM's default emulated display card (Apple Silicon: `virtio-ramfb-gl`). For these, specifying an integer value greater than 1 has no effect, so point clouds and other geometries are practically invisible when rendered with PCL Viewer. To remove the incompatibility, disable GPU acceleration from UTM's Virtual Machine settings in "Display" → "Emulated Display Card". Most cards whose name does not contain `-gl` work, but `virtio-ramfb` is recommended for M1+ Macintosh [3].

## Outstanding Issues

`CMakeLists.txt` contains references to additional files that could not be found or installed. [1] suggests to simply `touch` these files, but this workaround would just suppress the warnings without providing any real benefit, so I did not follow it.

- Missing files: `vtkParseOGLExt-7.1`, `libvtkRenderingPythonTkWidgets.so`

## Resources

1. https://github.com/dgrzech/sobfu/issues/15
2. https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/245
3. https://github.com/utmapp/UTM/discussions/5075