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
    return 10*n + (x**2 - 10*np.cos(2*math.pi*x)) + (y**2 - 10*np.cos(2*np.pi*y))

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

    child = parent1_first_chunk + parent2_second_chunk

    return child

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
    population_size = 10
    individual_size = 8
    generations = []
    costs = []
    crossover_rate = 0.05
    mutation_prob = 0.05
    max_iterations = 1000

    # initiate random population
    generation = [''.join(random.choices(['0', '1'], k=individual_size)) for _ in range(population_size)]
    generations.append(generation)

    # loop until max iterations
    for _ in range(max_iterations):
                  
        # evaluate individuals
        decoded_individuals = list(map(binary2xy, generation))
        generation_cost = list(map(target_function, decoded_individuals))
        costs.append(generation_cost)

        # perform selection 
        probabilities = [1/max((cost/sum(generation_cost)), 1) for cost in generation_cost]
        generation = random.choices(generation, k=population_size, weights=probabilities)

        # parent selection
        parents = [(random.choices(generation, k=2)) for _ in range(population_size)]
        
        # perform crossover
        if random.uniform(0, 1) <= crossover_rate:
            generation = [crossover_parents(ps, individual_size) for ps in parents]
        else:
            generation = [ps[0] for ps in parents]

        # perform mutation
        generation = [mutate(idv, mutation_prob) for idv in generation] 
        
        generations.append(generation)

    return generations, costs

if __name__ == "__main__":
    generations, costs = genetic_optimize(rosenbrock)
    min_costs = [min(c) for c in costs]
    max_costs = [max(c) for c in costs]
#    plt.figure(figsize=(12, 12))
    plt.plot(min_costs, label='min_gen_cost')
    plt.plot(max_costs, label='max_gen_cost')
    plt.legend()
    plt.show()


