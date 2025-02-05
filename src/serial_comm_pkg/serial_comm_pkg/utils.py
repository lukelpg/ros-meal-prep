import time

def retry_on_failure(func, retries=3, delay=1):
    """
    Helper function to retry a function call on failure.
    """
    attempt = 0
    while attempt < retries:
        try:
            return func()
        except Exception as e:
            print(f"Error occurred: {e}. Retrying...")
            time.sleep(delay)
            attempt += 1
    raise Exception("Max retries reached. Operation failed.")
