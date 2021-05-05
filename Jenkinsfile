@Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

defineProperties()

def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("dynamic_stack_decider")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("dynamic_stack_decider_visualization")))
pipeline.execute()