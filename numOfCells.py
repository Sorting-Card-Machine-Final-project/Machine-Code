cards = 52
rps = 3
for i in range(2,40):
    folds = 0
    restOfCards = cards
    while restOfCards > 1:
        restOfCards = restOfCards / i
        folds += 1
    print(f"{i}: the number of folds is {folds}")

## The Number of cells in our project will be 4 cells!