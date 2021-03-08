from tqdm import tqdm
import random
import pickle
import numpy as np
from multiprocessing import Pool
import matplotlib.pyplot as plt 

from Playground import Playground, QApplication
from NeuralNetwork import NeuralNetwork

HIDDEN_SIZE = 8
POPULATION_SIZE = 10
app = QApplication([])

def create_individual():
    nn = NeuralNetwork(HIDDEN_SIZE)
    WeightsH, BiasesH, WeightsO, BiasesO = nn.get_weights_biases()
    individual = [*WeightsH.ravel(), *WeightsO.ravel(), *BiasesH.ravel(), *BiasesO.ravel()]
    return individual


def init_generation(population_size):
    generation = []
    for _ in range(population_size):
        individual = create_individual()
        generation.append(individual)

    return generation 

def check_if_point_in_circle(point, center, radius):
    if point[0] <= center[0] + radius[0] and point[0] >= center[0] - radius[0]:
        if point[1] <= center[1] + radius[0] and point[1] >= center[1] - radius[0]:
            return True

    return False

def evaluate_episode(individual, episode_length_frames):

    reward = 0
    playground = Playground()

    # transfer NN weights
    first_layer_final_index = playground.nn.HiddenSize * playground.nn.InputSize
    second_layer_final_index = playground.nn.HiddenSize * playground.nn.OutputSize
    playground.nn.update_weights_biases(weightsH = np.array(individual[:first_layer_final_index]).reshape(playground.nn.WeightsH.shape),
                                        biasesH = np.array(individual[first_layer_final_index + second_layer_final_index : first_layer_final_index + second_layer_final_index + playground.nn.HiddenSize]).reshape(playground.nn.BiasesH.shape),
                                        weightsO = np.array(individual[first_layer_final_index : first_layer_final_index + second_layer_final_index]).reshape(playground.nn.WeightsO.shape),
                                        biasesO = np.array(individual[first_layer_final_index + second_layer_final_index + playground.nn.HiddenSize : first_layer_final_index + second_layer_final_index + playground.nn.HiddenSize + playground.nn.OutputSize]).reshape(playground.nn.BiasesO.shape))

    # init coins
    divisions = 30
    x = np.linspace(0, playground.SCREEN_WIDTH, divisions)
    y = np.linspace(0, playground.SCREEN_HEIGHT, divisions)
    X, Y = np.meshgrid(x, y)
    coin_coords = list(zip(X.ravel(), Y.ravel()))
    
    for _ in range(episode_length_frames):
        pos, collision = playground.playground_AI_update_flow(1)

        if collision:
            return -5
        else:
            for i, coin in enumerate(coin_coords):
                colected = check_if_point_in_circle(coin, pos, playground.robot.rsize)
                if colected:
                    reward += 1
                    coin_coords.pop(i)
    
    return reward

def run_parallel_evaluation(evaluation_arguments):
    pool = Pool(processes=len(evaluation_arguments))
    return pool.starmap(evaluate_episode, evaluation_arguments)

def crossover_parents(parents, individual_size):

    parent1, parent2 = parents
    crossover_index = random.randint(1, individual_size - 1)

    parent1_first_chunk = parent1[:crossover_index]
    parent1_second_chunk = parent1[crossover_index:]
    parent2_first_chunk = parent2[:crossover_index]
    parent2_second_chunk = parent2[crossover_index:]

    child1 = parent1_first_chunk + parent2_second_chunk
    child2 = parent2_first_chunk + parent1_second_chunk

    return child1, child2

def mutate(individual, p):

    original_individual = [i for i in individual]# individual.split().copy()
    mutated_individual = original_individual.copy()
    for i, gene in enumerate(original_individual):
        if random.uniform(0, 1) <= p:
            mutated_individual[i] *= random.uniform(-2, 2)

    return mutated_individual

def main():

    # initialize the generation
    generation = init_generation(POPULATION_SIZE)

    # initiate other parameters
    crossover_rate = 0.05
    mutation_rate = 0.05
    max_iterations = 50
    episode_length_frames = 50
    costs = []
    generations = []

    for _ in tqdm(range(max_iterations)):
        # evaluate individuals
        # generation_cost = [evaluate_episode(individual, episode_length_frames) for individual in generation] # parallelize
        evaluation_arguments = [[individual.copy(), episode_length_frames] for individual in generation]
        # print(evaluation_arguments)
        generation_cost = run_parallel_evaluation(evaluation_arguments)
        costs.append(generation_cost)
        total_generation_cost = sum(generation_cost)

        if total_generation_cost == 0:
            break

        ranked_generation = [idv for fitness, idv in sorted(zip(generation_cost, generation), reverse=True)]
        ranked_costs = sorted(generation_cost)
        
        # perform selection
        probabilities = [cost/total_generation_cost for cost in ranked_costs]
        cumulative_probabilities = np.cumsum(probabilities)
        
        # parent selection
        new_generation = []
        for _ in generation:
            for j, p in enumerate(cumulative_probabilities):
                if random.random() <= p:
                    new_generation.append(ranked_generation[j])
                    break
        
        # parent pairing
        new_generation_parent_pairs = []
        for i in range(0, len(new_generation), 2):
            new_generation_parent_pairs.append((new_generation[i], new_generation[i+1]))
            if i == len(new_generation) - 3:
                break
        
        # perform crossover
        new_generation = []
        for parents in new_generation_parent_pairs:
            if random.uniform(0, 1) <= crossover_rate:
                new_generation.extend(crossover_parents(parents, individual_size=len(generation[0])))
            else:
                new_generation.extend(parents)

        # perform mutation
        generation = [mutate(idv, mutation_rate) for idv in new_generation] 
        generations.append(generation)

    return generations, costs

if __name__ == "__main__":
    generations, costs = main()
    min_costs = [min(cg) for cg in costs]
    max_costs = [max(cg) for cg in costs]
    plt.plot(min_costs, label='min')
    plt.plot(max_costs, label='max')
    plt.legend()
    plt.show()

    with open('weights.pkl', 'wb') as f:
        pickle.dump(generations, f)

    with open('costs.pkl', 'wb') as f:
        pickle.dump(costs, f)