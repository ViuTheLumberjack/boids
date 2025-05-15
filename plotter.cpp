//
// Created by viu on 05/05/2025.
//

#include <fstream>
#include <filesystem>
#include <format>
#include <vector>
#include <matplot/matplot.h>

int main()
{
    std::filesystem::path resultFolder = std::filesystem::current_path();
    resultFolder /= "results";

    int experiments = 100;
    int boids = 100;
    int visualRange = 100;
    std::vector<int> maxV = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<int> threads = {12, 13};

    // create the result array, each entry is in microseconds
    auto result = std::vector<std::vector<std::vector<double>>>(3);
    for (int i = 0; i < 3; i++)
    {
        result[i] = std::vector<std::vector<double>>(maxV.size());
        for (int j = 0; j < maxV.size(); j++)
        {
            result[i][j] = std::vector<double>(threads.size());
        }
    }

    std::ifstream fileAos(resultFolder / std::format("aos_seq_{}_{}.txt", boids, visualRange));
    float aosSeqMean = 0;
    float aosSeqRead = 0;
    for (int i = 0; i < experiments; i++)
    {
        fileAos >> aosSeqRead;
        aosSeqMean += aosSeqRead;
    }
    aosSeqMean = aosSeqMean / experiments;
    std::cout << "AOS SEQ mean: " << aosSeqMean << std::endl;
    fileAos.close();

    // read the file
    for (int t = 0; t < threads.size(); t++)
    {
        for (int m = 0; m < maxV.size(); m++)
        {
            int thread = threads[t];
            int max = maxV[m];

            // std::ifstream aosoaFile(resultFolder / std::format("aosoa_par_{}_{}_{}_{}.txt", thread, boids, visualRange, max));
            // std::ifstream soaFile(resultFolder / std::format("soa_par_{}_{}_{}.txt", thread, boids, visualRange));
            std::ifstream aosFile(resultFolder / std::format("aos_par_{}_{}_{}.txt", thread, boids, visualRange));

            float meanAosoa = 0;
            float meanSoa = 0;
            float meanAos = 0;

            float readAosoa = 0;
            float readSoa = 0;
            float readAos = 0;

            for (int i = 0; i < experiments; i++)
            {
                aosFile >> readAos;
                // soaFile >> readSoa;
                // aosoaFile >> readAosoa;

                meanAos += readAos;
                meanSoa += readSoa;
                meanAosoa += readAosoa;
            }

            meanAosoa = meanAosoa / experiments;
            meanSoa = meanSoa / experiments;
            meanAos = meanAos / experiments;

            result[0][0][thread] = meanAos;
            result[1][0][thread] = meanSoa;
            result[2][max - 1][thread] = meanAosoa;

            // aosoaFile.close();
            // soaFile.close();
            aosFile.close();
        }
    }

    for (int t = 0; t < threads.size(); t++)
    {
        int thread = threads[t];
        for (int j = 0; j < maxV.size(); j++)
        {
            // result[2][j][thread] = result[2][j][thread] / result[0][0][thread];
        }
        std::cout << "AOS " << thread << " threads time: " << result[0][0][thread] << std::endl;
        result[0][0][thread] = aosSeqMean / result[0][0][thread];
        std::cout << "AOS " << thread << " threads mean: " << result[0][0][thread] << std::endl;
    }

    // Graph the result, on the x axis the number of threads, on the y axis the speedup
    // for each maxV, plot a line

    using namespace matplot;

    auto p1 = plot(threads, result[0][0]);
    p1->display_name("AOS");

    title("Parallel Speedup of AOS with " + std::to_string(boids) + " boids and " +
          std::to_string(visualRange) + " visual range");
    ::matplot::legend({});

    show();

    return 0;
}
