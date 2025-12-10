# Data Model: Book Content Structure

## Chapter Entity
- **name**: String - The name/title of the chapter
- **slug**: String - URL-friendly identifier for the chapter
- **purpose**: String - The main purpose and focus of the chapter
- **learningObjectives**: Array<String> - Specific learning objectives for the chapter
- **technicalDepth**: Enum - Level of technical depth (beginner, intermediate, advanced)
- **frameworks**: Array<String> - Robotics frameworks integrated (ROS 2, Gazebo, Isaac, VSLAM, VLA)
- **contentSections**: Array<ContentSection> - Sections within the chapter
- **assignments**: Array<Assignment> - Assignments associated with the chapter
- **assessments**: Array<Assessment> - Assessment materials for the chapter
- **figures**: Array<Figure> - Figures, diagrams, and tables for the chapter
- **inClassDemos**: Array<InClassDemo> - In-class demonstration materials

## ContentSection Entity
- **title**: String - Title of the section
- **content**: String - Markdown content of the section
- **type**: Enum - Type of content (explanation, code, theory, practice)
- **order**: Number - Order of the section within the chapter
- **requiresCodeExecution**: Boolean - Whether this section requires code execution examples

## Assignment Entity
- **title**: String - Title of the assignment
- **description**: String - Detailed description of the assignment
- **difficulty**: Enum - Difficulty level (beginner, intermediate, advanced)
- **estimatedTime**: Number - Estimated time to complete in minutes
- **requirements**: Array<String> - Technical requirements for the assignment
- **learningOutcomes**: Array<String> - Learning outcomes from completing the assignment

## Assessment Entity
- **type**: Enum - Type of assessment (quiz, practical, project, exam)
- **questions**: Array<Question> - Questions included in the assessment
- **passingScore**: Number - Minimum score required to pass (0-100)
- **duration**: Number - Duration of assessment in minutes

## Question Entity
- **text**: String - The question text
- **type**: Enum - Type of question (multiple-choice, short-answer, practical)
- **options**: Array<String> - Options for multiple choice questions
- **correctAnswer**: String - The correct answer
- **explanation**: String - Explanation of the correct answer

## Figure Entity
- **title**: String - Title of the figure
- **description**: String - Description of the figure
- **type**: Enum - Type of figure (diagram, chart, screenshot, illustration)
- **filePath**: String - Path to the figure file
- **caption**: String - Caption for the figure
- **altText**: String - Alternative text for accessibility

## InClassDemo Entity
- **title**: String - Title of the demo
- **description**: String - Description of the demo
- **materialsNeeded**: Array<String> - Materials required for the demo
- **setupInstructions**: String - Instructions for setting up the demo
- **executionSteps**: Array<String> - Steps to execute the demo
- **estimatedTime**: Number - Estimated time for the demo in minutes

## Validation Rules
- Each chapter MUST have a unique slug
- Each chapter MUST have at least one learning objective
- Each chapter MUST specify at least one robotics framework integration
- Each assignment MUST have a difficulty level
- Each assessment MUST have at least one question
- Each figure MUST have alt text for accessibility
- Content sections within a chapter MUST have unique order values

## State Transitions
- Chapter: draft → review → approved → published
- Assignment: design → review → approved → available
- Assessment: design → review → approved → active