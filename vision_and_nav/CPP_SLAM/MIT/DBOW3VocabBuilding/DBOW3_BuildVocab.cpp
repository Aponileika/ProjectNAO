#include "DBOW3_BuildVocab.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>


#define PANTO_DBOW_BUILD_VOCAB true

namespace fs = std::filesystem;
const fs::path Directory = "/Users/Jonathan/Programmering/FIA/PANTOPILOT/vision_and_nav/CPP_SLAM/datasets/DBOW3_training_images/Images";
constexpr u64 PANTO_DBOW_QUERY_RESULTS = 5;
constexpr u64 PANTO_DBOW_NEAR_FRAME_DISTANCE = 40;
constexpr u64 PANTO_DBOW_MEDIUM_FRAME_DISTANCE = 200;

int main(void)
{
    std::cout << "OpenCV threads: "
          << cv::getNumThreads()
          << '\n';
    std::vector<std::string> FilePaths;
    for(const fs::directory_entry& Entry : fs::directory_iterator(Directory))
    {
        if(!Entry.is_regular_file())
            continue;
        FilePaths.push_back(Entry.path().string());
    }
    std::sort(FilePaths.begin(), FilePaths.end());
    std::cout << "Number of images in dataset: " << FilePaths.size() << "\n";
    std::vector<std::string> DatasetNames;
    std::vector<u64> FrameIDs;
    for(const std::string& FilePath : FilePaths)
    {
        const std::string FileName = fs::path(FilePath).filename().string();
        const std::size_t FramePosition = FileName.rfind("_frame-");
        if(FramePosition == std::string::npos)
        {
            std::cerr << "Could not parse dataset/frame from: " << FileName << "\n";
            return 1;
        }
        const std::string DatasetName = FileName.substr(0, FramePosition);
        const std::size_t FrameIDBegin = FramePosition + 7;
        const std::size_t FrameIDEnd = FileName.find('.', FrameIDBegin);
        const u64 FrameID = std::stoull(
                FileName.substr(FrameIDBegin, FrameIDEnd - FrameIDBegin));
        DatasetNames.push_back(DatasetName);
        FrameIDs.push_back(FrameID);
    }
    const fp64 Threshold = OPENCV_AKAZETHRESHOLD;
    cv::Ptr<cv::AKAZE> Akaze = cv::AKAZE::create();
    Akaze->setThreshold(Threshold);
    Akaze->setNOctaves(OPENCV_AKAZE_NOCTAVES);
    Akaze->setNOctaveLayers(OPENCV_AKAZE_NOCTAVELAYERS);
    std::vector<cv::Mat> TrainingDescriptors;
    std::cout << "Sleeping for 5 seconds before extracting features\n";
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::chrono::steady_clock::time_point Begin = std::chrono::steady_clock::now();
    for(const std::string& ImagePath : FilePaths)
    {
        cv::Mat Image = cv::imread(ImagePath, cv::IMREAD_GRAYSCALE);
        if(Image.empty())
        {
            std::cerr << "Could not load image: " << ImagePath << "\n";
            return 1;
        }
        std::vector<cv::KeyPoint> KeyPoints;
        cv::Mat Descriptors;
        Akaze->detectAndCompute(
                Image,
                cv::noArray(),
                KeyPoints,
                Descriptors);
        if(Descriptors.empty())
        {
            std::cerr << "No descriptors extracted from: " << ImagePath << "\n";
            return 1;
        }
        TrainingDescriptors.push_back(Descriptors);
    }
    std::chrono::steady_clock::time_point End = std::chrono::steady_clock::now();
    std::chrono::duration DurationFeat(
            std::chrono::duration_cast<std::chrono::seconds>(End-Begin));
    std::chrono::duration<fp64> DurationCreateAndSave{};

#if PANTO_DBOW_BUILD_VOCAB

    DBoW3::Vocabulary Vocabulary(
            PANTO_DBOW_BRANCHING_FACTOR,
            PANTO_DBOW_DEPTH,
            DBoW3::TF_IDF,
            DBoW3::L1_NORM);

    Begin = std::chrono::steady_clock::now();
    Vocabulary.create(TrainingDescriptors);
    Vocabulary.save(PANTO_VocabFilePath);
    End = std::chrono::steady_clock::now();

    DurationCreateAndSave = End-Begin;

    std::cout << "Saved vocab to " << PANTO_VocabFilePath << "\n";
#endif

    DBoW3::Vocabulary SavedVocab(PANTO_VocabFilePath);
    Begin = std::chrono::steady_clock::now();
    DBoW3::Database DataBase(SavedVocab, false, 0);
    for(const cv::Mat& Descriptor : TrainingDescriptors)
        DataBase.add(Descriptor);
    End = std::chrono::steady_clock::now();
    std::chrono::duration DurationCreateDB(
            std::chrono::duration_cast<std::chrono::seconds>(End-Begin));

    std::vector<fp64> QueryTimes;
    std::vector<fp64> BestNonSelfScores;
    std::vector<fp64> ReturnedNonSelfScores;
    std::vector<fp64> BestSecondRatios;
    std::vector<fp64> SameDatasetScores;
    std::vector<fp64> DifferentDatasetScores;
    std::vector<fp64> SameDatasetFrameDistances;
    std::vector<fp64> NearFrameScores;
    std::vector<fp64> MediumFrameScores;
    std::vector<fp64> FarFrameScores;

    u64 SelfTop1Count{};
    u64 BestNonSelfSameDatasetCount{};
    u64 ValidBestNonSelfCount{};

    for(u64 QueryID{}; QueryID < TrainingDescriptors.size(); ++QueryID)
    {
        DBoW3::QueryResults Results;
        Begin = std::chrono::steady_clock::now();
        DataBase.query(
                TrainingDescriptors[QueryID],
                Results,
                PANTO_DBOW_QUERY_RESULTS);
        End = std::chrono::steady_clock::now();
        const fp64 QueryTime = std::chrono::duration<fp64, std::milli>(
                End-Begin).count();
        QueryTimes.push_back(QueryTime);

        if(!Results.empty() && Results.front().Id == QueryID)
        {
            ++SelfTop1Count;
        }

        fp64 BestScore{-1.0};
        fp64 SecondBestScore{-1.0};
        DBoW3::EntryId BestID{};

        for(const auto& Result : Results)
        {
            if(Result.Id == QueryID)
                continue;

            ReturnedNonSelfScores.push_back(Result.Score);
            if(DatasetNames[QueryID] == DatasetNames[Result.Id])
            {
                SameDatasetScores.push_back(Result.Score);
                const u64 FrameDistance = FrameIDs[QueryID] > FrameIDs[Result.Id]
                    ? FrameIDs[QueryID] - FrameIDs[Result.Id]
                    : FrameIDs[Result.Id] - FrameIDs[QueryID];

                SameDatasetFrameDistances.push_back(
                        static_cast<fp64>(FrameDistance));

                if(FrameDistance <= PANTO_DBOW_NEAR_FRAME_DISTANCE)
                {
                    NearFrameScores.push_back(Result.Score);
                }

                else if(FrameDistance <= PANTO_DBOW_MEDIUM_FRAME_DISTANCE)
                {
                    MediumFrameScores.push_back(Result.Score);
                }

                else
                {
                    FarFrameScores.push_back(Result.Score);
                }
            }

            else
            {
                DifferentDatasetScores.push_back(Result.Score);
            }

            if(Result.Score > BestScore)
            {
                SecondBestScore = BestScore;
                BestScore = Result.Score;
                BestID = Result.Id;
            }

            else if(Result.Score > SecondBestScore)
            {
                SecondBestScore = Result.Score;
            }
        }

        if(BestScore >= 0.0)
        {
            BestNonSelfScores.push_back(BestScore);
            ++ValidBestNonSelfCount;
            if(DatasetNames[QueryID] == DatasetNames[BestID])
            {
                ++BestNonSelfSameDatasetCount;
            }
        }

        if(SecondBestScore > 0.0)
        {
            BestSecondRatios.push_back(BestScore / SecondBestScore);
        }
    }

    fp64 MeanQueryTime{};
    fp64 MeanBestNonSelfScore{};
    fp64 MeanReturnedNonSelfScore{};
    fp64 MeanBestSecondRatio{};
    fp64 MeanSameDatasetScore{};
    fp64 MeanDifferentDatasetScore{};
    fp64 MeanSameDatasetFrameDistance{};
    fp64 MeanNearFrameScore{};
    fp64 MeanMediumFrameScore{};
    fp64 MeanFarFrameScore{};

    for(const fp64 Value : QueryTimes)
    {
        MeanQueryTime += Value;
    }

    for(const fp64 Value : BestNonSelfScores)
    {
        MeanBestNonSelfScore += Value;
    }

    for(const fp64 Value : ReturnedNonSelfScores)
    {
        MeanReturnedNonSelfScore += Value;
    }

    for(const fp64 Value : BestSecondRatios)
    {
        MeanBestSecondRatio += Value;
    }

    for(const fp64 Value : SameDatasetScores)
    {
        MeanSameDatasetScore += Value;
    }

    for(const fp64 Value : DifferentDatasetScores)
    {
        MeanDifferentDatasetScore += Value;
    }

    for(const fp64 Value : SameDatasetFrameDistances)
    {
        MeanSameDatasetFrameDistance += Value;
    }

    for(const fp64 Value : NearFrameScores)
    {
        MeanNearFrameScore += Value;
    }

    for(const fp64 Value : MediumFrameScores)
    {
        MeanMediumFrameScore += Value;
    }

    for(const fp64 Value : FarFrameScores)
    {
        MeanFarFrameScore += Value;
    }

    if(!QueryTimes.empty())
    {
        MeanQueryTime /= static_cast<fp64>(QueryTimes.size());
    }

    if(!BestNonSelfScores.empty())
    {
        MeanBestNonSelfScore /= static_cast<fp64>(BestNonSelfScores.size());
    }

    if(!ReturnedNonSelfScores.empty())
    {
        MeanReturnedNonSelfScore /= static_cast<fp64>(ReturnedNonSelfScores.size());
    }

    if(!BestSecondRatios.empty())
    {
        MeanBestSecondRatio /= static_cast<fp64>(BestSecondRatios.size());
    }

    if(!SameDatasetScores.empty())
    {
        MeanSameDatasetScore /= static_cast<fp64>(SameDatasetScores.size());
    }

    if(!DifferentDatasetScores.empty())
    {
        MeanDifferentDatasetScore /= static_cast<fp64>(DifferentDatasetScores.size());
    }

    if(!SameDatasetFrameDistances.empty())
    {
        MeanSameDatasetFrameDistance /= static_cast<fp64>(SameDatasetFrameDistances.size());
    }

    if(!NearFrameScores.empty())
    {
        MeanNearFrameScore /= static_cast<fp64>(NearFrameScores.size());
    }

    if(!MediumFrameScores.empty())
    {
        MeanMediumFrameScore /= static_cast<fp64>(MediumFrameScores.size());
    }

    if(!FarFrameScores.empty())
    {
        MeanFarFrameScore /= static_cast<fp64>(FarFrameScores.size());
    }

    fp64 QueryTimeVariance{};
    fp64 BestNonSelfScoreVariance{};
    fp64 ReturnedNonSelfScoreVariance{};
    fp64 BestSecondRatioVariance{};
    fp64 SameDatasetFrameDistanceVariance{};

    for(const fp64 Value : QueryTimes)
    {
        const fp64 Difference = Value - MeanQueryTime;
        QueryTimeVariance += Difference * Difference;
    }

    for(const fp64 Value : BestNonSelfScores)
    {
        const fp64 Difference = Value - MeanBestNonSelfScore;
        BestNonSelfScoreVariance += Difference * Difference;
    }

    for(const fp64 Value : ReturnedNonSelfScores)
    {
        const fp64 Difference = Value - MeanReturnedNonSelfScore;
        ReturnedNonSelfScoreVariance += Difference * Difference;
    }

    for(const fp64 Value : BestSecondRatios)
    {
        const fp64 Difference = Value - MeanBestSecondRatio;
        BestSecondRatioVariance += Difference * Difference;
    }

    for(const fp64 Value : SameDatasetFrameDistances)
    {
        const fp64 Difference = Value - MeanSameDatasetFrameDistance;
        SameDatasetFrameDistanceVariance += Difference * Difference;
    }

    if(!QueryTimes.empty())
    {
        QueryTimeVariance /= static_cast<fp64>(QueryTimes.size());
    }

    if(!BestNonSelfScores.empty())

        BestNonSelfScoreVariance /= static_cast<fp64>(BestNonSelfScores.size());



    if(!ReturnedNonSelfScores.empty())
    {
        ReturnedNonSelfScoreVariance /= static_cast<fp64>(ReturnedNonSelfScores.size());
    }

    if(!BestSecondRatios.empty())
    {
        BestSecondRatioVariance /= static_cast<fp64>(BestSecondRatios.size());
    }

    if(!SameDatasetFrameDistances.empty())
    {
        SameDatasetFrameDistanceVariance /= static_cast<fp64>(SameDatasetFrameDistances.size());
    }

    const fp64 QueryTimeStdDev = std::sqrt(QueryTimeVariance);
    const fp64 BestNonSelfScoreStdDev = std::sqrt(BestNonSelfScoreVariance);
    const fp64 ReturnedNonSelfScoreStdDev = std::sqrt(ReturnedNonSelfScoreVariance);
    const fp64 BestSecondRatioStdDev = std::sqrt(BestSecondRatioVariance);
    const fp64 SameDatasetFrameDistanceStdDev = std::sqrt(SameDatasetFrameDistanceVariance);

    const fp64 SelfTop1Rate = !TrainingDescriptors.empty() ? static_cast<fp64>(SelfTop1Count) / static_cast<fp64>(TrainingDescriptors.size())
        : 0.0;

    const fp64 BestNonSelfSameDatasetRate = ValidBestNonSelfCount > 0 ? static_cast<fp64>(BestNonSelfSameDatasetCount) / static_cast<fp64>(ValidBestNonSelfCount)
        : 0.0;

    std::cout << "\n========================================\n";
    std::cout << "DBoW3 diagnostic summary\n";
    std::cout << "========================================\n";
    std::cout << "\nDataset:\n";
    std::cout << "Images: " << FilePaths.size() << "\n";
    std::cout << "Query results requested: " << PANTO_DBOW_QUERY_RESULTS << "\n";
    std::cout << "\nTiming:\n";
    std::cout << "Getting features took: " << DurationFeat.count() << "[s]\n";

#if PANTO_DBOW_BUILD_VOCAB

    std::cout << "Creating and saving vocab took: "
        << DurationCreateAndSave.count() << "[s]\n";
#endif

    std::cout << "Creating DB took: " << DurationCreateDB.count() << "[s]\n";
    std::cout << "Mean query time: " << MeanQueryTime << "[ms]\n";
    std::cout << "Query time std dev: " << QueryTimeStdDev << "[ms]\n";

    std::cout << "\nGeneral result statistics:\n";
    std::cout << "Self Top-1 rate: " << SelfTop1Rate * 100.0 << "[%]\n";
    std::cout << "Mean best non-self score: " << MeanBestNonSelfScore << "\n";
    std::cout << "Best non-self score std dev: " << BestNonSelfScoreStdDev << "\n";
    std::cout << "Mean returned non-self score: " << MeanReturnedNonSelfScore << "\n";
    std::cout << "Returned non-self score std dev: " << ReturnedNonSelfScoreStdDev << "\n";
    std::cout << "Mean best/second ratio: " << MeanBestSecondRatio << "\n";
    std::cout << "Best/second ratio std dev: " << BestSecondRatioStdDev << "\n";

    std::cout << "\nSequence relationship statistics:\n";
    std::cout << "Best non-self match from same dataset: "
        << BestNonSelfSameDatasetRate * 100.0 << "[%]\n";
    std::cout << "Mean same-dataset score: " << MeanSameDatasetScore
        << " (" << SameDatasetScores.size() << " results)\n";
    std::cout << "Mean different-dataset score: " << MeanDifferentDatasetScore
        << " (" << DifferentDatasetScores.size() << " results)\n";
    std::cout << "Mean same-dataset frame distance: "
        << MeanSameDatasetFrameDistance << "\n";
    std::cout << "Same-dataset frame distance std dev: "
        << SameDatasetFrameDistanceStdDev << "\n";

    std::cout << "\nFrame-distance score statistics:\n";
    std::cout << "Near [0-" << PANTO_DBOW_NEAR_FRAME_DISTANCE
        << "] mean score: " << MeanNearFrameScore
        << " (" << NearFrameScores.size() << " results)\n";
    std::cout << "Medium [" << PANTO_DBOW_NEAR_FRAME_DISTANCE + 1
        << "-" << PANTO_DBOW_MEDIUM_FRAME_DISTANCE
        << "] mean score: " << MeanMediumFrameScore
        << " (" << MediumFrameScores.size() << " results)\n";
    std::cout << "Far [>" << PANTO_DBOW_MEDIUM_FRAME_DISTANCE
        << "] mean score: " << MeanFarFrameScore
        << " (" << FarFrameScores.size() << " results)\n";

    return 0;
}
