import matplotlib.pyplot as plt
import random
import numpy as np

def rosenbrock(coords):
    x, y = coords
    a = 0
    b = 1
    return (a-x)**2 + b*(y-x**2)**2

def rastrigin(coords):
    x, y = coords
    n = 2
    return 10*n + (x**2 - 10*np.cos(2*np.pi*x)) + (y**2 - 10*np.cos(2*np.pi*y))

def binary2xy(binary_number):

    # split into xy
    size = len(binary_number)//2
    x = binary_number[:size] 
    y = binary_number[size:]

    # first character will determine the xy sign
    character2sign_map = {'0': -1, '1': 1}
    x = character2sign_map[x[0]] * int(x[1:], 2) 
    y = character2sign_map[y[0]] * int(y[1:], 2) 

    return x,y

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
    mutation_map = {'0':'1', '1':'0'}
    for i, gene in enumerate(original_individual):
        if random.uniform(0, 1) <= p:
            mutated_individual[i] = mutation_map[gene]  

    return ''.join(mutated_individual)

def genetic_optimize(target_function):

    # initiate other parameters
    population_size = 20
    individual_size = 16
    generations = []
    costs = []
    crossover_rate = 0.02
    mutation_prob = 0.02
    max_iterations = 250

    # initiate random population
    generation = [''.join(random.choices(['0', '1'], k=individual_size)) for _ in range(population_size)]
    generations.append(generation)

    # loop until max iterations
    for _ in range(max_iterations):

        # evaluate individuals
        decoded_individuals = [binary2xy(individual) for individual in generation] 
        generation_cost = [target_function(dcd_individual) for dcd_individual in decoded_individuals]
        costs.append(generation_cost)
        total_generation_cost = sum(generation_cost)

        if total_generation_cost == 0:
            break

        ranked_generation = [idv for fitness, idv in sorted(zip(generation_cost, generation))]
        ranked_costs = sorted(generation_cost)
        
        # perform selection 
        generation_cost_probabilities = [cost/total_generation_cost for cost in ranked_costs]
        generation_cost_minimize = [1 - maxim_prob for maxim_prob in generation_cost_probabilities]
        probabilities = [cost/sum(generation_cost_minimize) for cost in generation_cost_minimize]
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
                new_generation.extend(crossover_parents(parents, individual_size))
            else:
                new_generation.extend(parents)

        # perform mutation
        generation = [mutate(idv, mutation_prob) for idv in new_generation] 
        generations.append(generation)

    return generations, costs


if __name__ == "__main__":
    # generations, costs = genetic_optimize(rosenbrock)
    generations, costs = genetic_optimize(rastrigin)
    min_costs = [min(c) for c in costs]
    max_costs = [max(c) for c in costs]
    median_costs = [np.median(c) for c in costs]
    #    plt.figure(figsize=(12, 12))
    plt.plot(min_costs, label='min_gen_cost')
    plt.plot(max_costs, label='max_gen_cost')
    plt.legend()
    plt.show()

    plt.plot(median_costs, label='median_gen_cost')
    plt.legend()
    plt.show()

