from PIL import Image
import sys, argparse, os
import numpy as np
import yaml as yaml
import pickle
from mc_func import *  

if __name__ == "__main__":

    ## Coment args declaration if you are using parameters_from_terminal()
    # args = {'ground_truth':'/home/xavi/Pictures/perfect_map_0.pgm',
    #         'map_path':'/home/xavi/Pictures/testing',
    #         'file_dest':'/home/xavi/Pictures/test.pkl',
    #         'metrics': ['error', 'accuracy'],
    #         'resolution':0.15,
    #         'align_mode':'intersection'}

    # Uncoment to read the parameters from the terminal
    args = parameters_from_terminal(sys.argv[0], sys.argv[1:], 
                                   descr='Compare maps with the ground truth.')

    if os.path.isdir(args['map_path']):
        paths_to_maps = [os.path.join(args['map_path'], f) for f in os.listdir(args['map_path'])
                        if f[-3:] == 'pgm']
    else:
        paths_to_maps = [args['map_path']]

    dicti = evaluate_maps(args['ground_truth'], paths_to_maps, args['resolution'],
                args['metrics'], args['align_mode'])

    print(dicti)

    with open(args['file_dest'], 'wb') as f:
        pickle.dump(dicti, f, 0)

    

    ## TESTING
    # truth_arr=np.random.randn(500,310)
    # map_arr=np.random.randn(400,300)
    # coord_truth=np.array([0,0])
    # coord_map=np.array([-30, -50])
    # al_truth_arr, al_map_arr=align_maps( truth_arr, map_arr, 
    #                                     coord_truth, coord_map, 1, 
    #                                     'union')
    
    # print(al_truth_arr.shape, al_map_arr.shape)
    # print(truth_arr.shape, map_arr.shape)