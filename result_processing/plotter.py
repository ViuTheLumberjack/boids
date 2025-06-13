from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import os

# "results_par_barrier",
folders = ["results_seq", "results_seq_kd",  "results_par_kd_barrier", "results_par_no_barrier", "results_par_kd_no_barrier"]

# open the times file and read the data
def read_log(file_path: os.path) -> pd.DataFrame:
    try:
        with open(file_path, 'r') as file:
            res_seq = ["seq", "seq_kd"]
            res_par = ["par_barrier", "par_no_barrier", "par_kd_barrier", "par_kd_no_barrier"]
            cols = ["folder", "parallel", "threads", "boids", "visual_range", "mean", "std", "min", "max"]
            df_total = pd.DataFrame(columns=cols)
            df = pd.DataFrame(columns=cols)
            data = file.readlines()
            title = "results_{}"
            count = 0

            for line in data:
                if "Running" in line and "boids" in line and "visual range" in line:
                    df_total = pd.concat([df_total, df])
                    count = 0
                    df = pd.DataFrame(columns=cols)
                    prototype_row = {
                        "folder": "",
                        "parallel": "par" in file_path,
                        "threads": 0,  # Default for sequential runs
                        "boids": line.split()[1],
                        "visual_range": line.split()[3],
                        "mean": 0.0,
                        "std": 0.0,
                        "min": 0.0,
                        "max": 0.0
                    }
                elif "Running" in line and "threads" in line:
                    count = 0
                    prototype_row["threads"] = int(line.split()[2])
                else:
                    prototype_row["parallel"] = prototype_row["threads"] > 0 
                    prototype_row["folder"] = title.format(res_seq[count] if prototype_row["parallel"] == False else res_par[count])
                    prototype_row["mean"] = float(line.split()[1])
                    df = pd.concat([df, pd.DataFrame([prototype_row])], ignore_index=True)
                    count += 1
            df_total = pd.concat([df_total, df])
            return df_total
    except FileNotFoundError:
        print(f"File {file_path} not found.")
        return pd.Series()

def read_csv(file_path: os.path) -> pd.Series:
    try:
        data = pd.read_csv(file_path, header=None, names=["time"])
        return data
    except FileNotFoundError:
        print(f"File {file_path} not found.")
        return pd.DataFrame()
    
def compare_speedup(data: pd.DataFrame, title: str, xlabel: str, ylabel: str, save_path: str):
    for boids in data["boids"].unique():
        for visual_range in data["visual_range"].unique():
            plt.figure(figsize=(10, 6))
            for folder in data["folder"].unique():
                # Filter the data for the current folder, boids, and visual range
                filtered_data = data[(data["folder"] == folder) & (data["boids"] == boids) & (data["visual_range"] == visual_range) & (data["parallel"] == True)]
                if filtered_data.empty:
                    continue
                
                filtered_data = filtered_data.sort_values(by="threads")
                plt.plot(filtered_data["threads"], filtered_data["speedup"], marker='o', label=folder)
        
            plt.title(title.format(boids, visual_range))
            plt.xlabel(xlabel)
            plt.ylabel(ylabel)
            plt.xticks(np.arange(0, len(filtered_data["threads"]) + 1, 5))
            plt.legend()
            plt.grid(True)
            plt.savefig(save_path.format(boids, visual_range))
            plt.close()

def compare_times(data: pd.DataFrame, title: str, xlabel: str, ylabel: str, save_path: str):
    for boids in data["boids"].unique():
        for visual_range in data["visual_range"].unique():
            plt.figure(figsize=(10, 6))
            for folder in data["folder"].unique():
                # Filter the data for the current folder, boids, and visual range
                filtered_data = data[(data["folder"] == folder) & (data["boids"] == boids) & (data["visual_range"] == visual_range) & (data["parallel"] == True)]
                if filtered_data.empty:
                    continue
                
                filtered_data = filtered_data.sort_values(by="threads")
                plt.plot(filtered_data["threads"], filtered_data["mean"], marker='o', label=folder)
        
            plt.title(title.format(boids, visual_range))
            plt.xlabel(xlabel)
            plt.ylabel(ylabel)
            #plt.yscale('log')  # Use logarithmic scale for better visibility
            plt.xticks(np.arange(0, len(filtered_data["threads"]) + 1, 5))
            plt.legend()
            plt.grid(True)
            plt.savefig(save_path.format(boids, visual_range))
            plt.close()

def remove_outliers(df, column):
    # Find first and third quartile
    q1 = df[column].quantile(0.25)
    q3 = df[column].quantile(0.75)
    
    # Find interquartile range
    IQR = q3 - q1
    
    # Find lower and upper bound
    lower_bound = q1 - 1.5 * IQR
    upper_bound = q3 + 1.5 * IQR
    
    # Remove outliers
    return df[(df[column] > lower_bound) & (df[column] < upper_bound)]

def compare_sequential(data: pd.DataFrame, title: str, xlabel: str, ylabel: str, save_path: str):
    plt.figure(figsize=(10, 6))
    seq = plt.get_cmap('winter')
    kd = plt.get_cmap('summer')
    data = data.sort_values(by="visual_range")
    for i, visual_range in enumerate(data["visual_range"].unique()):
        
        for folder in [folder for folder in data["folder"].unique() if "seq" in folder]:
            # Filter the data for the current folder, boids, and visual range
            filtered_data = data[(data["folder"] == folder) & (data["visual_range"] == visual_range) & (data["parallel"] == False)]
            if filtered_data.empty:
                continue
            
            cap = len(data["visual_range"].unique()) * 2
            color = seq(i / cap) if "kd" not in folder else kd(i / len(data["visual_range"].unique()))
            filtered_data = filtered_data.sort_values(by="boids")
            plt.plot(filtered_data["boids"], filtered_data["mean"], marker='o', label=f"{folder} (VR: {visual_range})", color=color)
    
    plt.xlabel(xlabel)
    plt.xticks(filtered_data["boids"], rotation=45)
    plt.ylabel(ylabel)
    plt.yscale('log')  # Use logarithmic scale for better visibility
    plt.title(title.format(visual_range))
    plt.legend(loc='best')
    plt.savefig(save_path)
    plt.close()

if __name__ == "__main__":
    # get the path to this file 
    plot_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots")
    if not os.path.exists(plot_path):
        os.makedirs(plot_path)

    all_data = pd.read_csv(os.path.join(plot_path, "all_summary.csv"))

    if all_data.empty:
        all_data = pd.concat([all_data, read_log(os.path.join(os.path.dirname(os.path.abspath(__file__)), "boids"))], ignore_index=True)

        for folder in folders:
            folder_summary_df = pd.DataFrame()

            folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), folder)
            if not os.path.exists(folder_path):
                print(f"Folder {folder_path} does not exist.")
                continue

            # read the csv files in the folder
            for file_name in os.listdir(folder_path):
                if file_name.endswith(".txt") or file_name.endswith(".csv"):
                    current_file_path = os.path.join(folder_path, file_name)
                    
                    data = read_csv(current_file_path)
                    file_name = file_name[:-4] 

                    if data.empty:
                        continue

                    # data = remove_outliers(data, "time")

                    # extract the parameters from the file name
                    if "par" in file_name:
                        threads = int(file_name.split("par")[1].split("_")[1])
                        boids = int(file_name.split("par")[1].split("_")[2])
                        visual_range = int(file_name.split("par")[1].split("_")[3])
                    else:
                        threads = 0
                        boids = int(file_name.split("seq")[1].split("_")[1])
                        visual_range = int(file_name.split("seq")[1].split("_")[2])

                    # add the mean to a dataframe
                    mean = {
                        "folder": folder,
                        "parallel": "par" in file_name,
                        "threads": threads, # match first number if parallel else 0
                        "boids": boids,
                        "visual_range": visual_range,
                        "mean": float(data.mean()),
                        "std": float(data.std()),
                        "min": float(data.min()),
                        "max": float(data.max()),
                    }

                    new_row_df = pd.DataFrame([mean])
                    folder_summary_df = pd.concat([folder_summary_df, new_row_df], ignore_index=True)


            folder_summary_df.to_csv(os.path.join(plot_path, f"{folder}_summary.csv"), index=False)
            all_data = pd.concat([all_data, folder_summary_df], ignore_index=True)
        # calculate speedup for parallel runs
        all_data["speedup"] = all_data.apply(
            lambda row: (all_data[(all_data["folder"] == ("results_seq" if "kd" not in row["folder"] else "results_seq_kd")) & (all_data["threads"] == 0) & (all_data["boids"] == row["boids"]) & (all_data["visual_range"] == row["visual_range"])]["mean"].values[0] if row["parallel"] else row["mean"]) / row["mean"],
            axis=1
        )

        all_data.to_csv(os.path.join(plot_path, "all_summary.csv"), index=False)

    #plot the times for each folder ond the same plot varying boids and visual ranges
    compare_speedup(
        data=all_data[all_data["parallel"] == True],
        title="Speedup for Boids {} with {} visual range",
        xlabel="Threads",
        ylabel="Speedup",
        save_path=os.path.join(plot_path, "speedup_comparison_{}_{}.png")
    )

    compare_times(
        data=all_data,
        title="Comparison of Times for Boids {} with {} visual range",
        xlabel="Threads",
        ylabel="Time (seconds)",
        save_path=os.path.join(plot_path, "comparison_times_{}_{}.png")
    ) 

    compare_sequential(
        data=all_data,
        title="Comparison of SEQ Times",
        xlabel="Boids",
        ylabel="Time (seconds)",
        save_path=os.path.join(plot_path, "comparison_sequential.png")
    )
