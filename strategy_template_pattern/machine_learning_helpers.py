#!/usr/bin/python3
import numpy as np
import pandas as pd

from sklearn.metrics import (make_scorer, confusion_matrix)
from sklearn.model_selection import KFold, cross_val_score 
from sklearn.ensemble import (RandomForestClassifier, 
                              GradientBoostingClassifier)
from sklearn.linear_model import SGDClassifier, RidgeClassifier
from sklearn.base import clone

from lightgbm import LGBMClassifier 
from xgboost import XGBClassifier
from bayes_opt import BayesianOptimization


class MachineLearningHelper():

    def __init__(self, num_init_points: int, num_optimisation_iterations: int):
        self.optimisation_iterations = num_optimisation_iterations
        self.num_init_points = num_init_points
  
    #### ---------------------------------- HELPER FUNCTIONS --------------------------------------------------------------------

    def evaluate_classifier(self, df: pd.DataFrame, estimator):
        n_splits = 4
        X = df.drop(df.columns[-1], axis = 1).values
        Y = df.drop(df.columns[:-1], axis = 1).values.ravel()
        weighted_acc_scorer = make_scorer(self.weighted_acc, greater_is_better=True)
    
        #### Sometimes exception is thrown: (Need to address this issue)
        try:
            cval = cross_val_score(estimator, X, Y, scoring=weighted_acc_scorer, cv=n_splits, n_jobs = -1)
            return cval.mean()
        except: 
            
            kf = KFold(n_splits, shuffle = True, random_state=1)
            split_scores = []

            for train_index, test_index in kf.split(X):
                train_x, test_x = X[train_index], X[test_index]
                train_y, test_y = Y[train_index], Y[test_index]
                model_ = clone(estimator)
                model_.fit(train_x,train_y)
                preds = model_.predict(test_x)
                classwise_acc = self.weighted_acc(test_y, preds)
                split_scores.append(classwise_acc)

            return round(sum(split_scores) / n_splits, 4)

    #### ---------------------------------- LOSS FUNCTION --------------------------------------------------------------------

    def weighted_acc(self, y_true, y_pred):
        y_true, y_pred = np.array(y_true), np.array(y_pred)
        cm = confusion_matrix(y_true, y_pred)
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
        return np.nanmean(np.diagonal(cm))
    
    #### ---------------------------------- OPTIMISERS --------------------------------------------------------------------
    
    @staticmethod
    def round_params(max_params: dict) -> dict:
        
        for param in max_params['params']:
            if param == 'num_leaves':
                max_params['params'][param] = int(round(max_params['params'][param]))
            elif param == 'learning_rate':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'n_estimators':
                max_params['params'][param] = int(round(max_params['params'][param]))
            elif param == 'min_child_samples':
                max_params['params'][param] = int(round(max_params['params'][param]))
            elif param == 'subsample':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'colsample_bytree':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'reg_alpha':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'reg_lambda':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'max_depth':
                max_params['params'][param] = int(round(max_params['params'][param]))
            elif param == 'bootstrap':
                max_params['params'][param] = bool(round(max_params['params'][param]))
            elif param == 'min_samples_split':
                max_params['params'][param] = int(round(max_params['params'][param]))
            elif param == 'min_samples_leaf':
                max_params['params'][param] = int(round(max_params['params'][param]))
            elif param == 'max_features':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'tol':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'fit_intercept':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'gamma':
                max_params['params'][param] = round(max_params['params'][param], 2)
            elif param == 'alpha':
                max_params['params'][param] = round(max_params['params'][param], 2)

        return max_params

    # -------   RandomForest optimiser   ----------------------------------------------------------------------

    def optimise_rf_clf(self, data: pd.DataFrame) -> dict:
        
        def train_bayes_opt_rforest(n_estimators, max_depth, min_samples_split, min_samples_leaf, max_features, bootstrap):
            params_ = {
                'n_estimators': int(round(n_estimators)),
                'max_depth': int(round(max_depth)),
                'min_samples_split': int(round(min_samples_split)),
                'min_samples_leaf': int(round(min_samples_leaf)),
                'max_features': max_features,
                'bootstrap': bool(round(bootstrap))
            }
               
            mdl = RandomForestClassifier(**params_)
            return self.evaluate_classifier(data, mdl)

        bounds = {
            'n_estimators': (50, 300),            # Number of trees in the forest
            'max_depth': (3, 15),                 # Maximum depth of the tree
            'min_samples_split': (2, 20),         # Minimum number of samples required to split an internal node
            'min_samples_leaf': (1, 20),          # Minimum number of samples required to be at a leaf node
            'max_features': (0.5, 1.0),           # Number of features to consider when looking for the best split
            'bootstrap': (0, 1)                   # Method for sampling data points (0 for False, 1 for True)
        }
        
        optimizer = BayesianOptimization(f=train_bayes_opt_rforest, pbounds = bounds, random_state=1)
        optimizer.maximize(init_points=self.num_init_points, n_iter=self.optimisation_iterations)
        result = MachineLearningHelper.round_params(optimizer.max)
        
        print("Final result:", result)
        return result, RandomForestClassifier(**result['params'])
    
    # -------   GradientBoost optimiser   ----------------------------------------------------------------------

    def optimise_gradboost_clf(self, data: pd.DataFrame) -> dict:
        
        def train_bayes_opt_gradboost(n_estimators, learning_rate, max_depth, min_samples_split, min_samples_leaf, subsample, max_features):
            params_ = {
                'n_estimators': int(round(n_estimators)),
                'learning_rate': learning_rate,
                'max_depth': int(round(max_depth)),
                'min_samples_split': int(round(min_samples_split)),
                'min_samples_leaf': int(round(min_samples_leaf)),
                'subsample': subsample,
                'max_features': max_features
            }
            mdl = GradientBoostingClassifier(**params_)
            return self.evaluate_classifier(data, mdl)
        
        bounds = {
            'n_estimators': (50, 300),            # Number of boosting stages to be run
            'learning_rate': (0.01, 0.3),         # Step size shrinkage
            'max_depth': (3, 15),                 # Maximum depth of the individual estimators
            'min_samples_split': (2, 20),         # Minimum number of samples required to split an internal node
            'min_samples_leaf': (1, 20),          # Minimum number of samples required to be at a leaf node
            'subsample': (0.5, 1.0),              # Fraction of samples to be used for fitting the individual base learners
            'max_features': (0.5, 1.0)            # Number of features to consider when looking for the best split
        }
        optimizer = BayesianOptimization(f=train_bayes_opt_gradboost, pbounds = bounds, random_state=1)
        optimizer.maximize(init_points=self.num_init_points, n_iter=self.optimisation_iterations)
        result = MachineLearningHelper.round_params(optimizer.max)
        print("Final result:", result)
        return result, GradientBoostingClassifier(**result['params'])


    # -------   LightGBM optimiser   ----------------------------------------------------------------------

    def optimise_lightgbm_clf(self, data: pd.DataFrame) -> dict:
        
        def train_bayes_opt_lightgbm(num_leaves, learning_rate, n_estimators, min_child_samples, subsample, colsample_bytree, reg_alpha, reg_lambda):
            params_ = {
                'num_leaves': int(round(num_leaves)),
                'learning_rate': learning_rate,
                'n_estimators': int(round(n_estimators)),
                'min_child_samples': int(round(min_child_samples)),
                'subsample': subsample,
                'colsample_bytree': colsample_bytree,
                'reg_alpha': reg_alpha,
                'reg_lambda': reg_lambda,
                'verbose': -1
            }
            mdl = LGBMClassifier(**params_)
            return self.evaluate_classifier(data, mdl)
        
        bounds = {
            'num_leaves': (20, 150),              # Maximum number of leaves in one tree
            'learning_rate': (0.01, 0.3),         # Step size shrinkage
            'n_estimators': (50, 300),            # Number of boosting rounds
            'min_child_samples': (10, 50),         # Minimum number of data needed in a child (leaf)
            'subsample': (0.5, 1.0),              # Subsample ratio of the training instances
            'colsample_bytree': (0.5, 1.0),       # Subsample ratio of columns when constructing each tree
            'reg_alpha': (0, 1),                  # L1 regularization term on weights
            'reg_lambda': (0, 1)                  # L2 regularization term on weights
        }
        optimizer = BayesianOptimization(f=train_bayes_opt_lightgbm, pbounds = bounds, random_state=1)
        optimizer.maximize(init_points=self.num_init_points, n_iter=self.optimisation_iterations)
        result = MachineLearningHelper.round_params(optimizer.max)
        print("Final result:", result)
        return result, LGBMClassifier(**result['params'])

    # -------   SGD optimiser   ----------------------------------------------------------------------

    def optimise_sgb_clf(self, data: pd.DataFrame) -> dict:
        
        def train_bayes_opt_sgb(n_estimators, learning_rate, max_depth, min_samples_split, min_samples_leaf, subsample, max_features):
            params_ = {
                'n_estimators': int(round(n_estimators)),
                'learning_rate': learning_rate,
                'max_depth': int(round(max_depth)),
                'min_samples_split': int(round(min_samples_split)),
                'min_samples_leaf': int(round(min_samples_leaf)),
                'subsample': subsample,
                'max_features': max_features
        }
            mdl = GradientBoostingClassifier(**params_)
            return self.evaluate_classifier(data, mdl)
        
        bounds = {
            'n_estimators': (50, 300),            # Number of boosting stages to be run
            'learning_rate': (0.01, 0.3),         # Step size shrinkage
            'max_depth': (3, 15),                 # Maximum depth of the individual estimators
            'min_samples_split': (2, 20),         # Minimum number of samples required to split an internal node
            'min_samples_leaf': (1, 20),          # Minimum number of samples required to be at a leaf node
            'subsample': (0.5, 1.0),              # Fraction of samples to be used for fitting the individual base learners
            'max_features': (0.5, 1.0)            # Number of features to consider when looking for the best split
        }

        optimizer = BayesianOptimization(f=train_bayes_opt_sgb, pbounds = bounds, random_state=1)
        optimizer.maximize(init_points=self.num_init_points, n_iter=self.optimisation_iterations)
        result = MachineLearningHelper.round_params(optimizer.max)
        print("Final result:", result)
        return result, GradientBoostingClassifier(**result['params'])


    # -------   Ridge optimiser   ----------------------------------------------------------------------

    def optimise_ridge_clf(self, data: pd.DataFrame) -> dict:
        
        def train_bayes_opt_ridge(alpha, tol, fit_intercept):
            params_ = {
                'alpha': alpha,
                'tol': tol,
                'fit_intercept': bool(round(fit_intercept))
            }

            mdl = RidgeClassifier(**params_)
            return self.evaluate_classifier(data, mdl)
    
        
        bounds = {
            'alpha': (0.01, 10),
            'tol': (1e-5, 1e-1),
            'fit_intercept': (0, 1)
        }
        optimizer = BayesianOptimization(f=train_bayes_opt_ridge, pbounds = bounds, random_state=1)
        optimizer.maximize(init_points=self.num_init_points, n_iter=self.optimisation_iterations)
        result = MachineLearningHelper.round_params(optimizer.max)
        print("Final result:", result)
        return result, RidgeClassifier(**result['params'])

    # -------   XGBoost optimiser   ----------------------------------------------------------------------

    def optimise_xgb_clf(self, data: pd.DataFrame) -> dict:
    
        def train_bayes_opt_xgb(n_estimators, learning_rate, max_depth, min_child_weight, subsample, colsample_bytree, gamma, reg_alpha, reg_lambda):
            params_ = {
                'n_estimators': int(round(n_estimators)),
                'learning_rate': learning_rate,
                'max_depth': int(round(max_depth)),
                'min_child_weight': int(round(min_child_weight)),
                'subsample': subsample,
                'colsample_bytree': colsample_bytree,
                'gamma': gamma,
                'reg_alpha': reg_alpha,
                'reg_lambda': reg_lambda
            }
            mdl = XGBClassifier(**params_)
            return self.evaluate_classifier(data, mdl)

        
        bounds = {
                   'n_estimators': (50, 300),          # Number of boosting rounds
                    'max_depth': (3, 15),               # Maximum depth of a tree
                    'learning_rate': (0.01, 0.3),       # Step size shrinkage
                    'gamma': (0, 0.5),                  # Minimum loss reduction
                    'subsample': (0.5, 1.0),            # Subsample ratio of the training instances
                    'colsample_bytree': (0.5, 1.0),     # Subsample ratio of columns when constructing each tree
                    'reg_alpha': (0, 1),                # L1 regularization term on weights
                    'reg_lambda': (0, 1),                # L2 regularization term on weights
                    'min_child_weight': (1, 10)
                }
        optimizer = BayesianOptimization(f=train_bayes_opt_xgb, pbounds = bounds, random_state=1)
        optimizer.maximize(init_points=self.num_init_points, n_iter=self.optimisation_iterations)
        result = MachineLearningHelper.round_params(optimizer.max)
        print("Final result:", result)
        return result, XGBClassifier(**result['params'])