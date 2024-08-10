#!/usr/bin/python3
import argparse
import pandas as pd
from scipy.stats import pearsonr, t
from sklearn.preprocessing import LabelEncoder
from sklearn.decomposition import PCA

from sklearn.preprocessing import LabelEncoder
from sklearn.decomposition import PCA
from machine_learning_helpers import MachineLearningHelper


class ElectromagneticFrequencyClassifier():

    def __init__(self, file_dir: str = ""):
        self.filedir = file_dir
        self.df = pd.DataFrame()
        self.pearson_coeff_threshold = 0.2
        self.pca_num_components = 3
        self.classifier = None

    '''
        Rename target variable and encode it
    '''
    def preprocess_data(self, df: pd.DataFrame) -> pd.DataFrame:
        df.rename(columns={df.columns[-1]:"object_type"}, inplace=True)
        le = LabelEncoder()
        le.fit(df.object_type.unique().tolist())
        # Transform the target variable
        df['object_type_encoded'] = le.transform(df['object_type'])
        df.drop(columns=['object_type'], inplace=True)
        df[df.columns[:-1]] = df[df.columns[:-1]].apply(lambda x: (x - x.mean()) / x.std())
        return df
    
    '''
        For many columns, reduce number of indicators to avoid curse of dimensionality
    '''

    def filter_columns_with_pearson_coefficient(self, df: pd.DataFrame, pearson_coeff: float) -> pd.DataFrame:

        def is_significant_correlation(r, n, alpha=0.05):
            # Calculate the t-statistic
            t_stat = r * (n - 2)**0.5 / ((1 - r**2)**0.5)
            # Calculate the p-value
            p_value = t.sf(abs(t_stat), df=n-2) * 2  # two-tailed test
            return p_value < alpha
        
        n = len(df)
        alpha = 0.05
        target_column = df.columns[-1]
        # Calculate Pearson correlation coefficients and filter based on significance and practical threshold
        significant_correlations = {}
        for column in df.columns:
            if column != target_column:
                r, _ = pearsonr(df[column], df[target_column])
                if is_significant_correlation(r, n, alpha) and abs(r) >= pearson_coeff:
                    significant_correlations[column] = r

        # Convert to DataFrame and sort
        correlations_df = pd.DataFrame.from_dict(significant_correlations, orient='index', columns=['pearson_correlation'])
        correlations_df["absolute_corr"] = abs(correlations_df.pearson_correlation)
        correlations_df = correlations_df.sort_values(by='absolute_corr', ascending=False)
        cols = correlations_df.index.values.tolist()
        cols.append(df.columns[-1])
        df = df[cols]
        return df
    
    def reduce_dimensionality_with_pca(self, df: pd.DataFrame, num_components: int):
        pca = PCA(n_components=num_components)
        pcaColList = [("PC" + str(x)) for x in range(num_components)]
        pcaX = pca.fit_transform(df[df.columns[:-1]])
        df_pca = pd.DataFrame(pcaX, columns=pcaColList)
        df_pca[df.columns[-1]] = df[df.columns[-1]].values
        return df_pca
    

def main(args):

    # 0. Set parameters for classification
    emf_cl = ElectromagneticFrequencyClassifier(args.dir)
    emf_cl.pearson_coeff_threshold = 0.2
    emf_cl.pca_num_components = 3
    ml_helper = MachineLearningHelper(num_init_points = 10, num_optimisation_iterations = 30)

    # 1. load dataframe :
    filename = emf_cl.filedir + "data.csv" 
    df_raw = pd.read_csv(filename, sep=",")
    df_cleaned = emf_cl.preprocess_data(df_raw)
    df_cleaned = emf_cl.filter_columns_with_pearson_coefficient(df_cleaned, pearson_coeff=emf_cl.pearson_coeff_threshold)
    df_cleaned = emf_cl.reduce_dimensionality_with_pca(df_cleaned, num_components=emf_cl.pca_num_components)
    
    # 2. Start various classification model training with Bayesian hyperparamater optimisation
    #       - return results and model with hyperparameters 
    classifiers_dict = {}
    print("Optimising random_forest params...")
    classifiers_dict['random_forest'], model    = ml_helper.optimise_rf_clf(data = df_cleaned)
    classifiers_dict['random_forest']['model']  = model

    print("Optimising lightgbm params...")
    classifiers_dict['lightgbm'], model         = ml_helper.optimise_lightgbm_clf(data = df_cleaned)
    classifiers_dict['lightgbm']['model']       = model

    print("Optimising grad_boost params...")
    classifiers_dict['grad_boost'], model       = ml_helper.optimise_gradboost_clf(data = df_cleaned)
    classifiers_dict['grad_boost']['model']     = model

    print("Optimising ridge params...")
    classifiers_dict['ridge'], model            = ml_helper.optimise_ridge_clf(data = df_cleaned)
    classifiers_dict['ridge']['model']          = model

    print("Optimising sgb params...")
    classifiers_dict['sgb'], model              = ml_helper.optimise_sgb_clf(data = df_cleaned)
    classifiers_dict['sgb']['model']            = model

    print("Optimising xgb params...")
    classifiers_dict['xgb'], model              = ml_helper.optimise_xgb_clf(data = df_cleaned)
    classifiers_dict['xgb']['model']            = model
    
    # Process results
    best_classifier = {}
    max_acc = 0.0
    print("\nFinal comparison:")
    for clf in classifiers_dict:
        result = classifiers_dict[clf]

        if result['target'] > max_acc:
            result['name'] = clf
            best_classifier = result
            max_acc = result['target']
        
        print(clf, result['target'])

    print("\nBest classifier is: ", best_classifier['name'], ' with score ', best_classifier['target'], 
            'and hyperparameters: ', best_classifier['params'])

    # 3. Save best classifier
    emf_cl.classifier = best_classifier

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('dir', type=str,
                        help='filenames directory with relative location to script')
    # Parse the command-line arguments
    arguments = parser.parse_args()
    main(arguments)
        
