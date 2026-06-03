import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=".")
    parser.add_argument("--save", type=bool, default=True, help="Save data in CSV files")
    parser.add_argument("--delta", type=int, default=100, help="Delta time to get data")
    parser.add_argument("--max-pts", type=int, default=200000, help="Max point rows to vizalize")
    parser.add_argument("--port", type=int, default=8050, help="Port for Dash visualisation")
    args = parser.parse_args()
