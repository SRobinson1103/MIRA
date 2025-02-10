#ifndef EXPORT_H
#define EXPORT_H

#ifdef _WIN32
#ifdef PROJECT_EXPORTS
#define MIRA_API __declspec(dllexport)
#else
#define MIRA_API __declspec(dllimport)
#endif
#else
#define MIRA_API 
#endif

#endif