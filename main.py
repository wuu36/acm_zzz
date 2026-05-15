import sys, os
from output_postProcessing import pyplot

def main():
    print("Hello from acmzzz!")
    print(f'{sys.argv=}')
    if len(sys.argv) > 1:
        if sys.argv[1] == "pyplot":
            print(111)
            pyplot.matplot_example_plot()
        elif sys.argv[1] == 'cplot':
            print(321)
    else:
        print("else")
        os.chdir(os.path.dirname(__file__))
        os.system('uv run streamlit run st_main.py')


if __name__ == "__main__":
    main()
