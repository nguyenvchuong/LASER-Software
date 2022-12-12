//
// Created by sergio on 16/05/19.
//

#include "../../include/Model.h"
#include "../../include/Tensor.h"

#include <numeric>
#include <iomanip>

int main() {
    // Load model with a path to the .pb file. 
    // An optional std::vector<uint8_t> parameter can be used to supply Tensorflow with
    // session options. The vector must represent a serialized ConfigProto which can be 
    // generated manually in python. See create_config_options.py.
    // Example:
    // const std::vector<uint8_t> ModelConfigOptions = { 0x32, 0xb, 0x9, 0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xb9, 0x3f, 0x20, 0x1 };
    // Model model("../model.pb", ModelConfigOptions);
    // /home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/usc_learning/learning/rllib/temp/saved_model.pb
    //Model model("../model.pb");
    std::cout << 's' << std::endl;
    // Model model("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //              "usc_learning/learning/rllib/temp11/model.pb"); //temp5/model/saved_model.pb");
    Model model("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
                 "usc_learning/learning/rllib/exported/graph.pb"); //temp5/model/saved_model.pb");
    //Model model("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/usc_learning/learning/rllib/temp5/checkpoint/model.meta");
    std::cout << 'a' << std::endl;
    model.restore("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
                    "usc_learning/learning/rllib/exported/my_model");
    model.init();
    std::cout << 'i' << std::endl;
    std::vector<std::string> result = model.get_operations();
    // for (auto i: result)
    //     std::cout << i << '\n';
    //std::cout << result << std::endl;
    //std::cout << model.get_operations() << std::endl;

    // Tensor input_a{model, "input_a"};
    // Tensor input_b{model, "input_b"};
    // Tensor output{model, "result"};

    // std::vector<float> data(100);
    // std::iota(data.begin(), data.end(), 0);

    // input_a.set_data(data);
    // input_b.set_data(data);

    // model.run({&input_a, &input_b}, output);
    // for (float f : output.get_data<float>()) {
    //     std::cout << f << " ";
    // }
    // std::cout << std::endl;


    Tensor observation{model, "default_policy/observation"};
    Tensor action{model, "default_policy/cond/Merge"};
    Tensor action_1{model, "default_policy/cond_1/Merge"};

    Tensor is_training{model, "default_policy/is_training"};
    Tensor is_exploring{model, "default_policy/is_exploring"};
    Tensor timestep{model, "default_policy/timestep"};

    std::vector<float> data(48);
    std::iota(data.begin(), data.end(), 0);

    // std::vector<char> data1 {'false'} ;//(1, false);// {false};//(1);
    // //std::iota(data1.begin(), data1.end(), false);
    // std::vector<char> data2 {'true'}; //(1, true); //{true} ;//data2(1);
    // //std::iota(data2.begin(), data2.end(), true);
    // std::vector<int> data3 {1} ;

    observation.set_data(data);
    // is_training.set_data(data1);
    // is_exploring.set_data(data2);
    // timestep.set_data(data3);

    // std::vector<float> data2(16);
    // std::iota(data2.begin(), data2.end(), 0);
    // action.set_data(data2);

    // model.run({&observation}, action);
    //model.run(observation, {action, action_1});
    // model.run({&observation, &is_training, &is_exploring, &timestep}, {&action, &action_1});
    model.run({&observation}, {&action, &action_1});
    for (float f : action.get_data<float>()) {
        std::cout << f << " ";
    }
    for (float f : action_1.get_data<float>()) {
        std::cout << f << " ";
    }
    std::cout << std::endl;




    return 0;
}
