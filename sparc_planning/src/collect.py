import subprocess

def collect_sparc_files(files:list[str], output:str = './sparc_files/program.sp'):
    """fuse together multiple .sp files into one output file"""
    data = []
    # collects files into list of string lines
    for f in files:
        with open(f, 'r') as sparc_file:
            for line in sparc_file:
                data.append(line)
    # pastes them all into a single output file
    with open(output, 'w') as out_file:
        out_file.writelines(data)
    #todo split up files to reconstruct in the right order (sorts, predicates display)
    #todo screen for errors e.g. repeated sort names

def parse_sparc_ouput(output_file:str='sparc.out') -> list[str]:
    """extracts lines from sparc results file and cleans up notation and newlines"""
    with open(output_file, 'r') as f:
        lines = f.readlines()
        lines = [line.strip('?- ') for line in lines]
    return lines

def run_sparc(sparc_file:str = './sparc_files/program.sp', output_file:str='sparc.out'):
    process = subprocess.run(f'java -jar sparc.jar {sparc_file} -A > {output_file}', 
                         stdout=subprocess.PIPE, 
                         universal_newlines=True)

def main():
    pass

if __name__ == "__main__":
    main()