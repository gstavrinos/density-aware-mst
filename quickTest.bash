rosrun density_aware_mst test_damst_from_file /home/gstavrinos/catkin_ws/src/density-aware-mst/synthetic_data_sets/t7_2.txt /home/gstavrinos/damst_result.txt

python scripts/viz_res.py ~/damst_result.txt ~/damst_full_tree.txt ~/damst_removed_edges.txt true true false
