# Electromagnetic Frequency Classifier

## About

- The code primarily follows the Strategy Design Pattern combined with aspects of the Template Method Pattern.

### Strategy Design Pattern
- Definition: The Strategy Pattern allows a class to change its behavior by selecting from various algorithms (strategies) at runtime.
- Usage in the Code:
	- The MachineLearningHelper class is used to optimize different classifiers (like RandomForestClassifier, GradientBoostingClassifier, LGBMClassifier, etc.). Each optimization method (optimise_rf_clf, optimise_gradboost_clf, etc.) can be seen as a strategy that the ElectromagneticFrequencyClassifier class can employ for classification.
   - The selection of the best classifier after optimization can be seen as choosing the best strategy from a set of available strategies.,

### Template Method Pattern
- Definition: The Template Method Pattern defines the skeleton of an algorithm in a method, deferring some steps to subclasses. It allows subclasses to redefine certain steps of an algorithm without changing the algorithm’s structure.
- Usage in the Code:
   - The evaluate_classifier method in MachineLearningHelper serves as a template method. It encapsulates the logic for evaluating a model, with the exact implementation details of fitting the model and predicting handled by the specific machine learning classifier (e.g., RandomForest, LightGBM, etc.).
   - The specific steps in each classifier's optimization (like defining the parameters and bounds) are defined in the methods for each classifier type.,

### Additional Design Considerations:
- Encapsulation: The optimization of different classifiers is encapsulated within their respective methods in the MachineLearningHelper class, following good object-oriented principles.
- Single Responsibility Principle (SRP): Each class and method has a clear responsibility. For instance, ElectromagneticFrequencyClassifier handles data preprocessing and feature selection, while MachineLearningHelper focuses on model optimization.
- These design patterns help make the code modular, extensible, and easy to maintain, allowing new classifiers or optimization strategies to be added with minimal changes to the existing codebase.

## To Run 
(Assuming you have anaconda installed and are using a linux machine)

1. Activate conda (i.e source ~/anaconda3/bin/activate)

2. Run the bash script;
   source arkeus_test.sh

3. Set python script variables (if desired)
   1. See main function part *'0.1 Set input variables here if desired'*

4. The code is written in python.
      1. Be sure to give the location of the directory relative to the file being run.
      2. To run (example): **´python3 electromagnetic_frequency_classifier.py "./data/"**

## Further work

1. Simulation could continue for the following
   1. Varying the Pearson's coefficient
   2. Varying the limits on hyperparameter optimisation iterations
      1. Number of init-points
      2. Number of iterations
   3. Further classifiers
   4. More data to verify classifier performance in practice
2. Removing some code-duplication, improving exception handling

## Limitations

1. Depending on speed of classification required, some algorithms might be preferred over others but those included are relatively fast.

