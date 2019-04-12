import random

mode_list = ['c', 't', 'a', 't+', 'a+']

num_tests = 5

new_list = mode_list[:num_tests]
random.shuffle(new_list)

for i in range(num_tests):
    print(new_list[i])
    print(random.randint(0,2), random.randint(0,5))
    print(random.randint(0,2), random.randint(0,5))
    print(random.randint(0,2), random.randint(0,5))