@Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

defineProperties()

def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_field_rqt")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_gazebo_world")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_interactive_marker")).withoutDocumentation())
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_relative_rqt")))
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("humanoid_league_rviz_marker")).withoutDocumentation())
pipeline.execute()