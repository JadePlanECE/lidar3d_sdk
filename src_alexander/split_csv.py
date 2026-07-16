import os
import sys
import argparse

def split_csv(source_filepath, dest_folder, split_number):
    if not os.path.exists(dest_folder):
        os.makedirs(dest_folder)
    
    with open(source_filepath, 'r', encoding='utf-8') as f:
        # Count remaining data rows
        total_lines = sum(1 for _ in f)
        split_size = max(1, total_lines // split_number)
        if split_size > 45000000:
            sys.exit(f"[Error] Too much lines to process {split_size}\n")

        # Reset file pointer and skip header again
        f.seek(0)

        # Grab the header line
        header = f.readline()
        
        file_counter = 1
        line_counter = 0
        current_out_file = None
        
        for line in f:
            if line_counter % split_size == 0:
                if current_out_file:
                    current_out_file.close()
                
                output_path = os.path.join(dest_folder, f"split_part_{file_counter}.csv")
                print(f"Writing to: {output_path}")
                current_out_file = open(output_path, 'w', encoding='utf-8')
                current_out_file.write(header) # Write header to new file
                file_counter += 1
                
            current_out_file.write(line)
            line_counter += 1
            
        if current_out_file:
            current_out_file.close()
        
    print("Splitting complete!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument("--file", type=str, default="../data/alexander/points-lidar3.csv", help="Name of the CSV file from the lidar")
    parser.add_argument("--folder", type=str, default="../data/alexander/points-lidar3", help="Destination folder for split files")
    parser.add_argument("--split", type=float, default=3, help="Number of split wanted")
    args = parser.parse_args()

    split_csv(args.file, args.folder, args.split)
