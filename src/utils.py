import numpy as np

def clip(value,val_max,val_min):
    """
    値をクリッピングする関数

    :param value: 入力値
    :type value: listかnumpy.ndarray
    :param val_max: クリッピング後の最大値
    :param val_min: クリッピング後の最小値
    :return value_clipped: クリッピング後の値
    """

    value=np.array(value)
    value[value>val_max]=val_max
    value[value>val_min]=val_min

    return value


def map(input,input_max,input_min,output_max,output_min):
    """
    範囲を変換する関数
    """
    output=(input-input_min)*(output_max-output_min)/(input_max-input_min)+output_min
    return output