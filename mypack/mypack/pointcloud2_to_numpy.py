'''!
Функции для работы с PointCloud2.
'''

import numpy as np

from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2

## префикс для для имен мнимых полей необходимых для правильной конструкции сообщений разных типов
PREFIX = '__'

## взаимосвязь между полями типа PointField и numpy типами 
type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
## из PointField в numpy
pftype_to_nptype = dict(type_mappings)
## из numpy в PointField
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

## размеры (в байтах) типов PointField
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


def pointcloud2_to_dtype(cloud_msg):
    '''!
    Конвертирует список PointField в список элементов numpy.uint8
    @param cloud_msg: список типа PointField
    @return: возвращает список типа numpy.uint8
    '''
    offset = 0
    np_dtype_list = []
    for f in cloud_msg.fields:
        while offset < f.offset:
            # может быть доп. отступ между полями
            np_dtype_list.append(('%s%d' % (PREFIX, offset), np.uint8))
            offset += 1
        np_dtype_list.append((f.name, pftype_to_nptype[f.datatype]))
        offset += pftype_sizes[f.datatype]

    # может быть доп. отступ между полями
    while offset < cloud_msg.point_step:
        np_dtype_list.append(('%s%d' % (PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def arr_to_fields(cloud_arr):
    '''!
    Конвертирует список элементов numpy в список PointField 
    @param cloud_arr: список элементов numpy
    @return: возвращает список типа PointField
    '''
    fields = []
    for field_name in cloud_arr.dtype.names:
        np_field_type, field_offset = cloud_arr.dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        pf.count = 1 # is this ever more than one?
        fields.append(pf)
    return fields


def pointcloud2_to_array(cloud_msg, split_rgb=False, remove_padding=True):
    '''!
    Конвертирует ros2 PointCloud2 сообщение в список элементов numpy.uint8

    Изменяет форму возвращаемого массива (высота, ширина), даже если первоначальная высота равна 1
    
    @param cloud_msg: ros2 sensor_msgs.msg.PointCloud2
    @param split_rgb: если True, то вызывает метод split_rgb_field
    @param remove_padding: если True, то убирает отступы
    @return: возвращает массив (список) элементов numpy.uint8
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = pointcloud2_to_dtype(cloud_msg)

    # parse the cloud into an array
    cloud_arr = np.fromstring(bytes(cloud_msg.data), dtype_list)

    # remove the dummy fields that were added
    if remove_padding:
        cloud_arr = cloud_arr[
            [fname for fname, _type in dtype_list if not (fname[:len(PREFIX)] == PREFIX)]]

    if split_rgb:
        cloud_arr = split_rgb_field(cloud_arr)

    return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


def split_rgb_field(cloud_arr):
    '''!
    Разбивает массив с полем rgb типа float32 на массив с тремя полями r,g,b каждое из которых uint8
    @param cloud_arr: массив float32 с полем rgb 
    @returns: массив uint8 c полями r,g,b 
    '''
    rgb_arr = cloud_arr['rgb'].copy()
    rgb_arr.dtype = np.uint32
    r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb_arr & 255, dtype=np.uint8)

    # create a new array, without rgb, but with r, g, and b fields
    new_dtype = []
    for field_name in cloud_arr.dtype.names:
        field_type, field_offset = cloud_arr.dtype.fields[field_name]
        if not field_name == 'rgb':
            new_dtype.append((field_name, field_type))
    new_dtype.append(('r', np.uint8))
    new_dtype.append(('g', np.uint8))
    new_dtype.append(('b', np.uint8))
    new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

    # fill in the new array
    for field_name in new_cloud_arr.dtype.names:
        if field_name == 'r':
            new_cloud_arr[field_name] = r
        elif field_name == 'g':
            new_cloud_arr[field_name] = g
        elif field_name == 'b':
            new_cloud_arr[field_name] = b
        else:
            new_cloud_arr[field_name] = cloud_arr[field_name]
    return new_cloud_arr


def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
    '''!
    Преобразует массив с полями x,y,z в матрицу 3хN
    @param cloud_array: список элементов numpy
    @param dtype: тип элементов
    @returns: матрица 3хN
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(list(cloud_array.shape) + [3], dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']

    return points


def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    '''!
    Конвертирует ros2 PointCloud2 сообщение в матрицу 3хN значений координат из PointCloud2
    
    @param cloud_msg: ros2 sensor_msgs.msg.PointCloud2
    @param remove_nans: если True, то удалить NaN элементы
    @returns: матрица 3хN значений координат из PointCloud'а
    '''
    return get_xyz_points(pointcloud2_to_array(cloud_msg))
