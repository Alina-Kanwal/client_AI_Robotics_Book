# Physical AI & Humanoid Robotics

This is a comprehensive educational book on Physical AI and Humanoid Robotics, created using Spec-Kit Plus methodology and Claude Code, published with Docusaurus and deployed via GitHub + Vercel.

## About This Book

This book explores the emerging field of Physical AI and Embodied Intelligence — the science and engineering of creating AI systems that operate in and interact with the physical world through robotic bodies. It covers the complete robotics stack from communication frameworks to AI-powered perception and control.

## Book Structure

1. **Introduction**: Physical AI & Embodied Intelligence
2. **Module 1**: The Robotic Nervous System (ROS 2)
3. **Module 2**: The Digital Twin (Gazebo & Unity)
4. **Module 3**: The AI-Robot Brain (NVIDIA Isaac)
5. **Module 4**: Vision-Language-Action (VLA)
6. **Capstone**: Autonomous Humanoid (conceptual walkthrough)
7. **Conclusion**: Learning summary and future directions

## Target Audience

- Students with foundational AI and programming knowledge
- Learners exploring Physical AI and Embodied Intelligence
- Anyone interested in the intersection of AI and robotics

## Local Development

1. Navigate to the project directory:
   ```bash
   cd physical-ai-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm run dev
   ```

This will start a local development server at `http://localhost:3000` with hot reloading.

## Building for Production

To build the static site for production:

```bash
npm run build
```

The built site will be in the `build/` directory.

## Deployment to GitHub and Vercel

### Step 1: Push to GitHub

1. Initialize a Git repository:
   ```bash
   git init
   git add .
   git commit -m "Initial commit: Physical AI & Humanoid Robotics book"
   ```

2. Create a new repository on GitHub (e.g., `physical-ai-book`)

3. Add the remote and push:
   ```bash
   git remote add origin https://github.com/your-username/physical-ai-book.git
   git branch -M main
   git push -u origin main
   ```

### Step 2: Deploy to Vercel

1. Go to [vercel.com](https://vercel.com) and sign in with your GitHub account

2. Click "New Project" and select your `physical-ai-book` repository

3. Vercel will automatically detect this is a Docusaurus project and configure the build settings:
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Install Command: `npm install`

4. Click "Deploy" and Vercel will build and deploy your site

5. Your site will be available at a URL like `https://physical-ai-book.vercel.app`

### Step 3: Custom Domain (Optional)

If you want a custom domain:

1. In Vercel dashboard, go to your project settings
2. Navigate to "Domains" section
3. Add your custom domain and follow DNS configuration instructions

## Tech Stack

- **Spec-Kit Plus**: For planning and chapter generation
- **Claude Code**: For writing all book content
- **Docusaurus**: For publishing and documentation
- **Markdown/MDX**: Content format
- **GitHub**: Source control
- **Vercel**: Deployment platform

## Project Structure

```
physical-ai-book/
├── docs/                    # All book content
│   ├── intro/              # Introduction module
│   ├── module1/            # The Robotic Nervous System
│   ├── module2/            # The Digital Twin
│   ├── module3/            # The AI-Robot Brain
│   ├── module4/            # Vision-Language-Action
│   ├── capstone/           # Autonomous Humanoid
│   └── conclusion/         # Conclusion and summary
├── src/                    # Custom source files
│   └── css/               # Custom styles
├── static/                 # Static assets
│   └── img/               # Images and logos
├── package.json           # Dependencies and scripts
├── docusaurus.config.js   # Docusaurus configuration
├── sidebars.js            # Navigation configuration
└── README.md             # This file
```

## Contributing

To add or modify content:

1. Edit the Markdown files in the `docs/` directory
2. Update `sidebars.js` if you add new pages
3. Test changes locally with `npm run dev`
4. Commit and push to trigger deployment

## License

This project is open source and available under the [MIT License](LICENSE).

## Support

For support, please open an issue in the GitHub repository.