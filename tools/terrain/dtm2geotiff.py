"""
git clone git://github.com/OSGeo/gdal.git
cd gdal/gdal/swig/python/
git checkout tags/1.10.0
python3 setup.py install

git clone git://github.com/numpy/numpy.git
cd numpy/
git checkout v1.7.1
python3.3 setup.py install

git clone git://github.com/scipy/scipy.git
cd scipy/
git checkout v0.12.0
python3.3 setup.py install
"""
DTM_Z_MEAN=7
DTM_Z_MAX=9

def save_geotiff(filepath, raster32, width, height):
    import gdal
    driver = gdal.GetDriverByName('GTiff')
    ds = driver.Create(filepath, width, height, 1, gdal.GDT_Float32)
    ds.GetRasterBand(1).WriteArray( raster32 )

def flat2numpy(image32, width, height):
    import numpy
    tmp = [image32[row*width:(row+1)*width] for row in range(height)]
    return numpy.asarray(tmp, dtype=numpy.float32)

def save_numpy(filepath, numpy_array):
    import scipy.misc # DEBUG
    scipy.misc.imsave(filepath, numpy_array)

def read_dtm_ascii(filepath):
    with open(filepath, 'r') as f:
        buff = f.read().split()
    # see libdtm/src/io.c:45 in dtm_writeAsciiDtm()
    width = int(buff[16])
    height = int(buff[15])
    data = buff[24:]
    image = [float(data[DTM_Z_MEAN+i*10]) for i in range(len(data)//10)]
    assert(len(image) == width * height)
    return image,width,height

def main(argv=[]):
    if len(argv) < 3:
        print("usage: dtm2geotiff file.dtm file.tif")
    imgage32, width, height = read_dtm_ascii(argv[1])
    raster32 = flat2numpy(imgage32, width, height)
    save_geotiff(argv[2], raster32, width, height)
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))


header = '''%(dtm_typename)s
%(type)i %(storage)i %(cells)i %(bornes)i %(luminance)i
%(nim3d)i
%(x_orig)e %(y_orig)e
%(i_orig)i %(j_orig)i
%(scale_x)e %(scale_y)e
%(size_x)e %(size_y)e
%(nrow)i %(ncol)i
%(ncel)i
%(i_min)i %(i_max)i %(j_min)i %(j_max)i
%(z_min)e %(z_max)e'''

row = '%(state)i %(hstate)i %(may_be_visible)i %(npoints)i %(luminance)i %(first_date)i %(last_date)i %(z_mean)f %(sigma_z)f %(z_max)f'


