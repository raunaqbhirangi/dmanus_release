# D'Manus Interfacing Library
This library contains interfacing code for the D'Manus hand with large-area [ReSkin](https://reskin.dev) sensing

Useful links: [Paper](https://arxiv.org/abs/2210.15658) [Website](https://sites.google.com/view/dmanus) [Data](https://drive.google.com/drive/folders/1TixE6MkVUNsy48dUiikXmxKUE9AAuDjL?usp=share_link)

The library is built on top of a fork of the [dynamixel](https://github.com/vikashplus/dynamixel) library and the [ReSkin](https://github.com/raunaqbhirangi/reskin_sensor) library. We offer a [Gym](https://github.com/openai/gym) interface for simple and familiar interaction.

## Setup
1. Create a conda env using the provided env.yml file .
``` 
conda env create -f env.yml 
conda activate dmanus_env
```
2. Follow [instructions](https://github.com/vikashplus/dynamixel) to setup dynamixel SDK.
3. Run the collect_data.py file to run motor babble policy to collect data. Custom policies can be defined in `conf/policy` to initialize one of the provided policy classes in `policies.py`.

## Citation and Bibtex
Bhirangi, R., DeFranco, A., Adkins, J., Majidi, C., Gupta, A., Hellebrekers, T., & Kumar, V. (2022). All the Feels: A dexterous hand with large area sensing. arXiv preprint arXiv:2210.15658.

```
@article{bhirangi2022all,
  title={All the Feels: A dexterous hand with large area sensing},
  author={Bhirangi, Raunaq and DeFranco, Abigail and Adkins, Jacob and Majidi, Carmel and Gupta, Abhinav and Hellebrekers, Tess and Kumar, Vikash},
  journal={arXiv preprint arXiv:2210.15658},
  year={2022}
}
```