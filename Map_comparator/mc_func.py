from PIL import Image
import sys, argparse, os
import numpy as np
import yaml as yaml
import pickle
from collections import OrderedDict

def parameters_from_terminal( program_name, args, descr ):
    # Function that returns the arguments readed from the command line
    # stored in a dictionary

    parser = argparse.ArgumentParser(prog=program_name, description=descr)

    # Argument for the file location of the ground truth
    parser.add_argument('ground_truth', metavar='ground_truth', type=str,
                        help='Absolute path of the ground truth .pgm file')

    # Argument for the location of the maps to be tested
    # can be directory or a single file
    parser.add_argument('map_path', metavar='map_path', type=str,
                        help='File/directory absolute path of the evaluated maps')

    # Argument for the location of the file with the results
    parser.add_argument('file_dest', metavar='f_dest', type=str,
                        help='Absolute path to the file where to store the results')

    # Argument for decide how to align the two maps
    parser.add_argument('-a', '--align_mode', metavar='align_mode', type=str,
                        default='intersection',
                        help='Intersection or union of the maps, deafult intersection')

    # Argument for the resolution
    parser.add_argument('-r', '--res', type=float,
                        default=0.15, dest='resolution',
                        help='Resolution of the maps, deafult 0.15')

    # Argument for deciding if normalize or not
    parser.add_argument('-m', '--metrics', type=str,
                        nargs='+', default=['error', 'accuracy'],
                        help='Metrics to be computed, deafult [error, accuracy]')

    # Return a dictionary of the arguments
    return vars(parser.parse_args(args))

def align_maps( truth_arr, map_arr, coord_truth, coord_map,resolution, mode):

    offset_bott_left=((coord_map-coord_truth)/resolution).astype(int)
    # print(offset_bott_left)
    offset_up_right=np.array([len(truth_arr[0]),len(truth_arr)]) - offset_bott_left - np.array([len(map_arr[0]),len(map_arr)],
                            dtype=int)
    # print(offset_up_right)

    if mode == 'intersection':  # Cropping needed
        # Cropping colums left
        if offset_bott_left[0] < 0:
            map_arr = map_arr[:,abs(offset_bott_left[0]):]
        elif offset_bott_left[0] > 0:
            truth_arr = truth_arr[:,offset_bott_left[0]:]
        # Cropping colums right
        if offset_up_right[0] < 0:
            map_arr = map_arr[:,:offset_up_right[0]]
        elif offset_up_right[0] > 0:
            truth_arr = truth_arr[:,:-offset_up_right[0]]
        #Cropping rows bottom
        if offset_bott_left[1] < 0:
            map_arr = map_arr[:offset_bott_left[1],:]
        elif offset_bott_left[1] > 0:
            truth_arr = truth_arr[:-offset_bott_left[1],:]
        #Cropping rows up
        if offset_up_right[1] < 0:
            map_arr = map_arr[abs(offset_up_right[1]):,:]
        elif offset_up_right[1] > 0:
            truth_arr = truth_arr[offset_up_right[1]:,:]

    elif mode == 'union':   # Padding needed
        truth_pad_up = truth_pad_bottom = truth_pad_left = truth_pad_right = 0
        map_pad_up = map_pad_bottom = map_pad_left = map_pad_right = 0

        # Padding colums left
        if offset_bott_left[0] < 0:
            truth_pad_left = abs(offset_bott_left[0])
        elif offset_bott_left[0] > 0:
            map_pad_left = offset_bott_left[0]
        # Padding colums right
        if offset_up_right[0] < 0:
            truth_pad_right = abs(offset_up_right[0])
        elif offset_up_right[0] > 0:
            map_pad_right = offset_up_right[0]
        #Padding rows bottom
        if offset_bott_left[1] < 0:
            truth_pad_bottom = abs(offset_bott_left[1])
        elif offset_bott_left[1] > 0:
            map_pad_bottom = offset_bott_left[1]
        #Padding rows up
        if offset_up_right[1] < 0:
            truth_pad_up = abs(offset_up_right[1])
        elif offset_up_right[1] > 0:
            map_pad_up = offset_up_right[1]

        # Apply the padding to the matrix 
        truth_arr=np.pad(truth_arr, 
                            ((truth_pad_up, truth_pad_bottom), (truth_pad_left, truth_pad_right)),
                            'constant')
        map_arr=np.pad(map_arr, 
                        ((map_pad_up, map_pad_bottom), (map_pad_left, map_pad_right)), 
                        'constant')

    return truth_arr, map_arr

def compute_diff(truth_arr, map_arr, metrics):
    # It should be ordered to be able to work with the values as np arrays later
    res=OrderedDict.fromkeys(metrics)

    pixels=truth_arr.shape[0]*truth_arr.shape[1]
    
    error=np.sum(np.abs(truth_arr + (-1*map_arr)))

    accuracy = (pixels-error) / float(pixels)

    if 'error' in metrics:
        res['error'] = error

    if 'accuracy' in metrics:
        res['accuracy'] = accuracy

    if 'F1' in metrics or 'precision' in metrics or 'recall' in metrics:
        tp = np.sum((truth_arr==1) & (map_arr==1))  # True positives
        fp = np.sum((truth_arr==0) & (map_arr==1))  # False positives
        fn = np.sum((truth_arr==1) & (map_arr==0))  # False negatives

        precision = float(tp)/(tp+fp)
        recall = float(tp)/(tp+fn)
        
        if'precision' in metrics:
            res['precision'] = precision
        if 'recall' in metrics:
            res['recall'] = recall
        if 'F1' in metrics: 
            res['F1'] = 2*precision*recall/(precision+recall)
        
    return res

def evaluate_maps(path_truth, paths_maps, resolution, metrics, mode='intersection'):
    total = np.zeros((1,len(metrics)))
    cache = np.zeros((len(paths_maps),len(metrics)))

    fn = lambda x : 0 if x > 200 else 1 # Function to binarize the images
    truth_im = Image.open(path_truth)   # Read truth image
    truth_im = truth_im.point(fn, mode='1') # Binarize the image
    truth_arr = np.asarray(truth_im, dtype=np.uint8) # Convert it into an array

    # Getting the coordinates of the bottom left corner of the ground truth
    yaml_file_path = path_truth[:-3] + 'yaml'
    yaml_file = open(yaml_file_path)
    parsed_yaml_file = yaml.safe_load(yaml_file)
    coord_truth = np.array(parsed_yaml_file['origin'], dtype='float32')[:-1]
    
    for i in range(len(paths_maps)):
        map_im = Image.open(paths_maps[i])   # Read map image
        map_im = map_im.point(fn, mode='1') # Binarize the image
        map_arr = np.asarray(map_im, dtype=np.uint8) # Convert it into an array

        # Getting the coordinates of the bottom left corner of the map
        yaml_file_path = paths_maps[i][:-3] + 'yaml'
        yaml_file = open(yaml_file_path)
        parsed_yaml_file = yaml.safe_load(yaml_file)
        coord_map = np.array(parsed_yaml_file['origin'], dtype='float32')[:-1]

        # Align and resize the matrix to have the same shape
        # we can overwrite map_arr, but truth_arr should remain as the original
        al_truth_arr, map_arr = align_maps(truth_arr, map_arr,
                                            coord_truth, coord_map, resolution, mode)

        diference_dict = compute_diff(al_truth_arr, map_arr, metrics) 
        
        total += np.array(diference_dict.values())
        cache[i] = np.array(diference_dict.values())

    avg = total.squeeze(axis=0)/len(paths_maps)
    std_dev = np.sqrt(np.sum(np.power(cache-avg,2),axis=0) / len(paths_maps))
    
    res = {}
    for index in range(len(metrics)): 
        res['avg_' + metrics[index]] = avg[index]
        res['std_dev_' + metrics[index]] = std_dev[index]

    return res